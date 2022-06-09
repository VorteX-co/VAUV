// Copyright 2022 VorteX-co
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

#include "local_planning_node.hpp"
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
LocalPlanner::LocalPlanner(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  // Subscribers initialization
  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/swift/pose_gt", 10, std::bind(&LocalPlanner::stateCallback, this, _1));
  cmd_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/local_planning/cmd_waypoint", 1,
    std::bind(&LocalPlanner::pointCallback, this, _1));
  cmd_attitude_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/local_planning/cmd_attitude", 1,
    std::bind(&LocalPlanner::attitudeCallback, this, _1));
  // Publishers initialization
  local_plan_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/local_planning/plan", 1);
  /*****************************************************
   *   Parameters
   * ***************************************************/
  Eigen::VectorXd translation_constraints = vector_to_eigen(
    this->get_parameter("translation_constraints").as_double_array());
  Eigen::VectorXd rotation_constraints = vector_to_eigen(
    this->get_parameter("rotation_constraints").as_double_array());
  this->trajectory_generator_.set_params(translation_constraints, rotation_constraints);
  double radius_of_accepetance =
    this->get_parameter("radius_of_accepetance").as_double();
  double dt = this->get_parameter("dt").as_double();
  double kappa = this->get_parameter("kappa").as_double();
  double delta = this->get_parameter("delta").as_double();
  this->los_.set_params(radius_of_accepetance, dt, kappa, delta);
}
// =========================================================================
/* Callbacks*/
void LocalPlanner::pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference waypoint by fitting a trajectory from current translation
   * state [x,y,z,u,v,w]<ENU> to a desired waypoint [xd,yd,zd] with zero end
   * velocity. And setting a line of sight goal point.
   */
  controller_on_ = true;
  Vector12d state, reference;
  reference(0) = msg->x;
  reference(1) = msg->y;
  reference(2) = msg->z;
  state = state_;
  trajectory_generator_.generate_trajectory(state, reference);
  trajectory_start_stamp_ = this->get_clock()->now().seconds();
  los_.setpoint(Vector2d(msg->x, msg->y));
//  los_on_ = true;
  std::cout << " ###############################" << std::endl;
  // std::cout << "trajectory_clock  "<< trajectory_clock<< std::endl;
  std::cout << " ###############################" << std::endl;
  std::cout << reference << std::endl;
  std::cout << " ###############################" << std::endl;
  // std::cout << "trajectory_clock  "<< trajectory_clock<< std::endl;
  std::cout << " ###############################" << std::endl;
  std::cout << state << std::endl;
//  Eigen::Matrix<double, 6, 2> logging;
//  logging <<desired_pose, desired_velocity;
}
// =========================================================================
void LocalPlanner::attitudeCallback(
  const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference attitude by fitting a trajectory from current attitude
   * state [φ,θ,ψ,p,q,r]<ENU> to a desired atttide [φ,θ,ψ] with zero end
   * velocity.
   */
  controller_on_ = true;
  Vector12d reference;
  Vector6d pose_reference;

  pose_reference << state_(0, 0), state_(1, 0), state_(2, 0), msg->x, msg->y, msg->z;
  reference.segment<6>(0) = pose_reference;
  trajectory_generator_.generate_trajectory(state_, reference);
  trajectory_start_stamp_ = this->get_clock()->now().seconds();
  los_on_ = false;
}
// =========================================================================
void LocalPlanner::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  /* Handle state message
   */
  // XYZ positions
  Vector3d position;
  tf2::fromMsg(msg->pose.pose.position, position);
  state_.block<3, 1>(0, 0) = position;
  // Orientation
  Vector4d quaternion{
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  state_.block<3, 1>(3, 0) = quaternion_to_euler(quaternion);
  // Velocities [linear + angular]
  Vector6d v;
  tf2::fromMsg(msg->twist.twist, v);
  state_.block<6, 1>(6, 0) = v;
  // Sending local plan to the controller
  if (controller_on_) {
    // Real time evaluation of the generated trajectory
    double trajectory_clock =
      this->get_clock()->now().seconds() - trajectory_start_stamp_;
    Vector6d desired_pose, desired_velocity, desired_acceleration;
    trajectory_generator_.evaluate_generated_trajectory(
      trajectory_clock, desired_pose, desired_velocity, desired_acceleration);
    desired_state_.block<6, 1>(0, 0) = desired_pose;
    desired_state_.block<6, 1>(6, 0) = desired_velocity;
    if (los_on_) {
      // Computing Line-of-sight based heading and heading rate
      double psi_des = 0.0;
      double r_des = 0.0;
      Vector2d planner_position = state_.head<2>();
      los_.compute_heading(planner_position, psi_des, r_des);
      desired_state_(5) = psi_des;
      desired_state_(11) = r_des;
    }
    this->publish_local_plan(state_, desired_state_, desired_acceleration);
  }
}
// =========================================================================
void LocalPlanner::publish_local_plan(
  const Vector12d state, const Vector12d desired_state,
  const Vector6d feedforward_acc)
{
  // Publishing the output local plan as a stacked vector
  VectorXd plan(12 + 12 + 6);
  plan << state, desired_state, feedforward_acc;
  // Covnverting from Eigen::Vector to std::vector
  std::vector<float> plan_vector(plan.data(), plan.data() + plan.rows() * plan.cols());
  auto plan_msg = std_msgs::msg::Float32MultiArray();
  plan_msg.data = plan_vector;
  this->local_plan_pub_->publish(plan_msg);
}
// =========================================================================
Vector3d LocalPlanner::quaternion_to_euler(const Vector4d & quaternion)
{
  /* Attitude conversion from quaternion to Euler
   * Reference: Equation (2.58) Fossen 2011
   */
  double e1 = quaternion(0);
  double e2 = quaternion(1);
  double e3 = quaternion(2);
  double eta = quaternion(3);
  Eigen::Matrix<double, 3, 3> R;
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
inline Eigen::VectorXd LocalPlanner::vector_to_eigen(std::vector<double> v)
{
  // Covnverting from std::vector to Eigen::Vector
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}
// =========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"local_planning_node"});
  std::shared_ptr<LocalPlanner> node = std::make_shared<LocalPlanner>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
