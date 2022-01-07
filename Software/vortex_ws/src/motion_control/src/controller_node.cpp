// Copyright 2021 VorteX-co
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "controller_node.hpp"
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

/* Constructor */
// =========================================================================
Controller::Controller(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  // Subscribers initialization
  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/rexrov/pose_gt", 10, std::bind(&Controller::stateCallback, this, _1));
  cmd_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/controller/cmd_waypoint", 1,
    std::bind(&Controller::pointCallback, this, _1));
  cmd_attitude_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/controller/cmd_attitude", 1,
    std::bind(&Controller::attitudeCallback, this, _1));
  // Publishers initialization
  tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/rexrov/thruster_manager/input_stamped", 1);
  pwm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
    "/swift/thruster_manager/pwm", 1);
  /*****************************************************
   *  AUV parameters
   * ***************************************************/
  double mass = this->get_parameter("mass").as_double();
  double volume = this->get_parameter("volume").as_double();
  double dt = this->get_parameter("dt").as_double();
  double T = this->get_parameter("T").as_int();
  Eigen::VectorXd Ib =
    vector_to_eigen(this->get_parameter("Ib").as_double_array());
  Eigen::VectorXd Dlinear =
    vector_to_eigen(this->get_parameter("Dlinear").as_double_array());
  Eigen::VectorXd Dquad =
    vector_to_eigen(this->get_parameter("Dquad").as_double_array());
  Eigen::VectorXd Ma =
    vector_to_eigen(this->get_parameter("Ma").as_double_array());
  Eigen::VectorXd r_cog =
    vector_to_eigen(this->get_parameter("r_cog").as_double_array());
  Eigen::VectorXd r_cob =
    vector_to_eigen(this->get_parameter("r_cob").as_double_array());
  /*****************************************************
   * Controller parameters
   * ***************************************************/
  Eigen::VectorXd Q =
    vector_to_eigen(this->get_parameter("Q").as_double_array());
  Eigen::VectorXd R1 =
    vector_to_eigen(this->get_parameter("R1").as_double_array());
  Eigen::VectorXd R2 =
    vector_to_eigen(this->get_parameter("R2").as_double_array());
  Eigen::VectorXd tau_max =
    vector_to_eigen(this->get_parameter("tau_max").as_double_array());
  Eigen::VectorXd error_max =
    vector_to_eigen(this->get_parameter("error_max").as_double_array());
  // Setting the Controller parameters
  this->mpc_.set_params(mass, volume, T, Ib, r_cob, r_cog, Ma, Dlinear, Dquad,
    Q, R1, R2, tau_max, error_max, dt);
  /*****************************************************
   *  Guidance parameters
   * ***************************************************/
  Eigen::VectorXd translation_constraints = vector_to_eigen(
    this->get_parameter("translation_constraints").as_double_array());
  Eigen::VectorXd rotation_constraints = vector_to_eigen(
    this->get_parameter("rotation_constraints").as_double_array());
  double radius_of_accepetance =
    this->get_parameter("radius_of_accepetance").as_double();
  this->guidance_.set_params(translation_constraints, rotation_constraints,
    radius_of_accepetance);
  /*****************************************************
   *  Thrust allocation parameters
   * ***************************************************/
  // Number of thusters
  const int r = this->get_parameter("r").as_int();
  MatrixXd Tpose(6, r);
  for (int i = 0; i < r; i++) {
    // Getting thruster i pose paramter [T1 .. Tr]
    std::string Tindex = std::to_string(i + 1);
    Tpose.col(i) =
      vector_to_eigen(this->get_parameter("T" + Tindex).as_double_array());
  }
  const int k = this->get_parameter("k").as_double();
  VectorXd coeff_left = vector_to_eigen(
    this->get_parameter("rpm_to_pwm_coeff_left").as_double_array());
  VectorXd coeff_right = vector_to_eigen(
    this->get_parameter("rpm_to_pwm_coeff_right").as_double_array());
  allocator_.set_params(k, Tpose, coeff_left, coeff_right);
}
// =========================================================================
/* Callbacks*/
void Controller::pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference waypoint by fitting a trajectory from current translation
   * state [x,y,z,u,v,w]<ENU> to a desired waypoint [xd,yd,zd] with zero end
   * velocity. And setting a line of sight goal point.
   */
  controller_on_ = true;
  Vector6d pose_state = x_.head<6>();
  double psi_ref =
    guidance_.los_steering(Vector2d(x_.head<2>()), Vector2d(msg->x, msg->y));
  Vector6d pose_reference;
  pose_reference << msg->x, msg->y, msg->z, 0, 0, 0.0;
  guidance_.generate_trajectory(pose_state, pose_reference);
  trajectory_start_stamp_ = this->get_clock()->now().seconds();
}
// =========================================================================
void Controller::attitudeCallback(
  const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference attitude by fitting a trajectory from current attitude
   * state [φ,θ,ψ,p,q,r]<ENU> to a desired atttide [φ,θ,ψ] with zero end
   * velocity.
   */
  controller_on_ = true;
  Vector6d pose_state = x_.head<6>();
  Vector6d pose_reference;
  pose_reference << x_(0, 0), x_(1, 0), x_(2, 0), msg->x, msg->y, msg->z;
  guidance_.generate_trajectory(pose_state, pose_reference);
  trajectory_start_stamp_ = this->get_clock()->now().seconds();
}
// =========================================================================
void Controller::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  /* Handle state message
   * 1- Converting the Odometry msg to Eigen 12D vector for MPC calculations
   * 2- Evaluating the generated reference trajectory at current clock
   * 3- Calculating MPC control action then publising the control wrench
   */
  Vector3d position;
  tf2::fromMsg(msg->pose.pose.position, position);
  x_.block<3, 1>(0, 0) = position;
  Vector4d quaternion{
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  x_.block<3, 1>(3, 0) = quaternion_to_euler(quaternion);
  Vector6d velocity;
  tf2::fromMsg(msg->twist.twist, velocity);
  x_.block<6, 1>(6, 0) = velocity;
  if (controller_on_) {
    double trajectory_clock =
      this->get_clock()->now().seconds() - trajectory_start_stamp_;
    Vector6d desired_pose, desired_velocity, desired_acceleration;
    std::tie(desired_pose, desired_velocity, desired_acceleration) =
      guidance_.evaluate_desired_trajectory(trajectory_clock);

    x_desired_.block<6, 1>(0, 0) = desired_pose;
    x_desired_.block<6, 1>(6, 0) = desired_velocity;
    acc_desired_ = desired_acceleration;
    control_wrench_ = mpc_.action(x_, x_desired_, acc_desired_);
    this->publish_control_wrench();
    Eigen::VectorXd pwm = allocator_.wrench_to_pwm_thrusters(control_wrench_);
    this->publish_pwm(pwm);
  }
}
// =========================================================================
void Controller::publish_control_wrench()
{
  auto tau = geometry_msgs::msg::WrenchStamped();
  // The produced forces from the controller are in NED frame.
  tau.header.frame_id = "rexrov/base_link_ned";
  tau.wrench.force.x = control_wrench_(0);
  tau.wrench.force.y = control_wrench_(1);
  tau.wrench.force.z = control_wrench_(2);
  tau.wrench.torque.x = control_wrench_(3);
  tau.wrench.torque.y = control_wrench_(4);
  tau.wrench.torque.z = control_wrench_(5);
  tau.header.stamp = this->get_clock()->now();
  tau_pub_->publish(tau);
}
// =========================================================================
void Controller::publish_pwm(const Eigen::VectorXd & pwm)
{
  std::vector<int> pwm_vec(pwm.data(), pwm.data() + pwm.rows() * pwm.cols());
  auto pwm_msg = std_msgs::msg::Int32MultiArray();
  pwm_msg.data = pwm_vec;
  this->pwm_pub_->publish(pwm_msg);
}
// =========================================================================
Vector3d Controller::quaternion_to_euler(const Vector4d & quaternion)
{
  /* Attitude conversion from quaternion to Euler
   * Reference: Equation (2.58) Fossen 2011
   */
  double e1 = quaternion(0);
  double e2 = quaternion(1);
  double e3 = quaternion(2);
  double eta = quaternion(3);
  Matrix3d R;
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e3 * eta),
    2 * (e1 * e3 + e2 * eta), 2 * (e1 * e2 + e3 * eta),
    1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e1 * eta),
    2 * (e1 * e3 - e2 * eta), 2 * (e2 * e3 + e1 * eta),
    1 - 2 * (e1 * e1 + e2 * e2);
  Vector3d euler;
  // Pitch, treating singularity cases θ = ±90
  double den = sqrt(1 - R(2, 0) * R(2, 0));
  euler << atan2(R(2, 1), R(2, 2)), -atan(R(2, 0) / std::max(0.001, den)),
    atan2(R(1, 0), R(0, 0));

  return euler;
}
// =========================================================================
inline Eigen::VectorXd Controller::vector_to_eigen(std::vector<double> v)
{
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}
// =========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"controller_node"});
  std::shared_ptr<Controller> node = std::make_shared<Controller>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
