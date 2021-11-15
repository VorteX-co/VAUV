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

using std::placeholders::_1;

/* Constructor */

Controller::Controller(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  // Subscribers initialization
  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/swift/pose_gt", 1, std::bind(&Controller::stateCallback, this, _1));
  cmd_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/LQR/cmd_waypoint", 1, std::bind(&Controller::pointCallback, this, _1));
  cmd_attitude_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/LQR/cmd_attitude", 1,
    std::bind(&Controller::attitudeCallback, this, _1));
  cmd_roll_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/LQR/cmd_roll", 1, std::bind(&Controller::rollCallback, this, _1));
  cmd_pitch_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/LQR/cmd_pitch", 1, std::bind(&Controller::pitchCallback, this, _1));
  cmd_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/LQR/cmd_yaw", 1, std::bind(&Controller::yawCallback, this, _1));
  cmd_hold_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/LQR/cmd_hold", 1, std::bind(&Controller::holdCallback, this, _1));
  // Publishers initialization
  tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/swift/thruster_manager/input_stamped", 1);
  pwm_client_ =
    this->create_client<custom_ros_interfaces::srv::PWM>("control_pwm");
}
Vector3d Controller::q_to_euler(Vector4d quat)
{
  /* Attitude conversion from quaternion to Euler
   * Reference: Equation (2.58) Fossen 2011
   */
  double e1 = quat(0);
  double e2 = quat(1);
  double e3 = quat(2);
  double eta = quat(3);
  Matrix3d R;
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e3 * eta),
    2 * (e1 * e3 + e2 * eta), 2 * (e1 * e2 + e3 * eta),
    1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e1 * eta),
    2 * (e1 * e3 - e2 * eta), 2 * (e2 * e3 + e1 * eta),
    1 - 2 * (e1 * e1 + e2 * e2);
  Vector3d euler;
  // Pitch, treating singularity cases θ = ±90
  double den = sqrt(1 - R(2, 0) * R(2, 0));
  euler << atan2(R(2, 1), R(2, 2)), -atan(R(2, 0) / max(0.001, den)),
    atan2(R(1, 0), R(0, 0));

  return euler;
}
/* Callbacks*/
void Controller::pointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference waypoint by fitting a trajectory from current translation
   * state [x,y,z,u,v,w]<ENU> to a desired waypoint [xd,yd,zd] with zero end
   * velocity. And setting a line of sight goal point.
   */
  controller_on_ = true;
  control_mode_ = modes::point_tracking;
  Vector6d translation_state;
  translation_state << x_(0), x_(1), x_(2), x_(6), x_(7), x_(8);
  Vector6d translation_reference;
  translation_reference << msg->x, msg->y, msg->z, 0, 0, 0;
  trajectory_generator_.generate(translation_reference, translation_state,
    trajectory_types::translation);
  translation_duration_ = trajectory_generator_.translation3D_duration;
  translation_start_stamp_ = this->get_clock()->now().seconds();
  los_.setpoint(Vector2d(msg->x, msg->y));
}
void Controller::attitudeCallback(
  const geometry_msgs::msg::Point::SharedPtr msg)
{
  /* Handle reference attitude by fitting a trajectory from current attitude
   * state [φ,θ,ψ,p,q,r]<ENU> to a desired atttide [φ,θ,ψ] with zero end
   * velocity.
   */
  controller_on_ = true;
  control_mode_ = modes::attitude_tracking;
  Vector6d attitude_state;
  attitude_state << x_(3), x_(4), x_(5), x_(9), x_(10), x_(11);
  Vector6d attitude_reference;
  attitude_reference << msg->x, msg->y, msg->z, 0, 0, 0;
  trajectory_generator_.generate(attitude_reference, attitude_state,
    trajectory_types::rotation);
  rotation_duration_ = trajectory_generator_.rotation3D_duration;
  rotation_start_stamp_ = this->get_clock()->now().seconds();
}
void Controller::rollCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  /* Handle reference roll angle by fitting 1D trajectory from current roll
   * angle to a commanded roll angle.
   */
  controller_on_ = true;
  control_mode_ = modes::roll_tracking;
  Vector2d roll_state{x_(4), x_(10)};
  Vector2d roll_reference{msg->data, 0.0};
  trajectory_generator_.generate(roll_reference, roll_state,
    trajectory_types::rotation);
  rotation_duration_ = trajectory_generator_.rotation1D_duration;
  rotation_start_stamp_ = this->get_clock()->now().seconds();
}
void Controller::pitchCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  /* Handle reference pitch angle by fitting 1D trajectory from current pitch
   * angle to a commanded pitch angle.
   */
  controller_on_ = true;
  control_mode_ = modes::pitch_tracking;
  Vector2d pitch_state{x_(4), x_(10)};
  Vector2d pitch_reference{msg->data, 0.0};
  trajectory_generator_.generate(pitch_reference, pitch_state,
    trajectory_types::rotation);
  rotation_duration_ = trajectory_generator_.rotation1D_duration;
  rotation_start_stamp_ = this->get_clock()->now().seconds();
}

void Controller::yawCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  /* Handle reference yaw angle by fitting 1D trajectory from current yaw angle
   * to a commanded yaw angle.
   */
  controller_on_ = true;
  control_mode_ = modes::yaw_tracking;
  Vector2d yaw_state{x_(5), x_(11)};
  Vector2d yaw_reference{msg->data, 0.0};
  trajectory_generator_.generate(yaw_reference, yaw_state,
    trajectory_types::rotation);
  rotation_duration_ = trajectory_generator_.rotation1D_duration;
  rotation_start_stamp_ = this->get_clock()->now().seconds();
}
void Controller::holdCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  /* Handle hold command
   * changing the control mode
   * saving hold pose to be the desired state
   */
  controller_on_ = true;
  control_mode_ = modes::station_keeping;
  x_hold_.setZero();
  x_hold_.block<6, 1>(0, 0) = x_.head<6>();
}
void Controller::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  /* Handle state message
   * 1- Converting the Odometry msg to Eigen 12D vector for lqr calculations
   * 2- Evaluating the generated reference trajectory at current clock
   * 3- Calculating LQR control action then publising the control wrench
   */
  Vector3d position;
  tf2::fromMsg(msg->pose.pose.position, position);
  x_.block<3, 1>(0, 0) = position;
  Vector4d q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
  x_.block<3, 1>(3, 0) = q_to_euler(q);
  Vector6d velocity;
  tf2::fromMsg(msg->twist.twist, velocity);
  x_.block<6, 1>(6, 0) = velocity;
  if (controller_on_) {
    if (control_mode_ == modes::station_keeping) {
      /* Setting a hold pose with zero velocity and acceleration
       */
      x_desired_ = x_hold_;
      acc_desired_.setZero();
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();
    } else if (control_mode_ == modes::point_tracking) {
      /* Evaluting the  generated trajectory and current clock w.r.t the
       * starting time point. The output from the generator is
       * [translation_reference] 9D vector contains [x,y,z] desired position,
       * linear velocity and linear acceleration.
       */
      translation_clock_ =
        this->get_clock()->now().seconds() - translation_start_stamp_;
      Vector9d translation_reference =
        trajectory_generator_.get_translation3D_trajectory(
        translation_clock_);
      x_desired_.block<3, 1>(0, 0) = translation_reference.head<3>();
      x_desired_.block<3, 1>(6, 0) = translation_reference.block<3, 1>(3, 0);
      acc_desired_.block<3, 1>(0, 0) = translation_reference.tail<3>();
      /* Calculating a line of sight refence yaw angle position, angular
       * velocity and angular acceleration
       */
      Vector3d planner_pose{x_(0), x_(1), x_(5)};
      Vector3d planner_velocity{x_(6), x_(7), x_(11)};
      los_.update_state(planner_pose, planner_velocity);
      los_.calculate_reference();
      x_desired_(5) = los_.yaw_des;
      x_desired_(11) = los_.rd;
      acc_desired_(5) = los_.rdd;
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();
    } else if (control_mode_ == modes::attitude_tracking) {
      /* Evaluting the  generated trajectory and current clock w.r.t the
       * starting time point. The output from the generator is
       * [rotation_reference] 9D vector contains [r,p,y] desired angles, angular
       * velocities and angular accelerations.
       */
      rotation_clock_ =
        this->get_clock()->now().seconds() - rotation_start_stamp_;
      Vector9d rotation_reference =
        trajectory_generator_.get_rotation3D_trajectory(rotation_clock_);
      x_desired_.block<3, 1>(3, 0) = rotation_reference.head<3>();
      x_desired_.block<3, 1>(9, 0) = rotation_reference.block<3, 1>(3, 0);
      acc_desired_.block<3, 1>(3, 0) = rotation_reference.tail<3>();
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();

    } else if (control_mode_ == modes::roll_tracking) {
      /* Evaluting the generated roll trajectory at current clock w.r.t the
       * starting time point the output from the generator is 3D vector
       * contains [φ] desired position, angular velocity and angular
       * acceleration.
       */
      rotation_clock_ =
        this->get_clock()->now().seconds() - rotation_start_stamp_;
      Vector3d roll_desired =
        trajectory_generator_.get_rotation1D_trajectory(rotation_clock_);
      x_desired_(3) = roll_desired(0);
      x_desired_(9) = roll_desired(1);
      acc_desired_(3) = roll_desired(2);
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();
    } else if (control_mode_ == modes::pitch_tracking) {
      /* Evaluting the generated pitch trajectory at current clock w.r.t the
       * starting time point the output from the generator is 3D vectors
       * contains [θ] desired position, angular velocity and angular
       * acceleration.
       */
      rotation_clock_ =
        this->get_clock()->now().seconds() - rotation_start_stamp_;
      Vector3d pitch_desired =
        trajectory_generator_.get_rotation1D_trajectory(rotation_clock_);
      x_desired_(4) = pitch_desired(0);
      x_desired_(10) = pitch_desired(1);
      acc_desired_(4) = pitch_desired(2);
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();
    } else if (control_mode_ == modes::yaw_tracking) {
      /* Evaluting the  generated yaw trajectory at current clock w.r.t the
       * starting time point the output from the generator is 3D vector
       * contains [ψ] desired position, angular velocity and angular
       * acceleration.
       */
      rotation_clock_ =
        this->get_clock()->now().seconds() - rotation_start_stamp_;
      Vector3d yaw_desired =
        trajectory_generator_.get_rotation1D_trajectory(rotation_clock_);
      x_desired_(5) = yaw_desired(0);
      x_desired_(11) = yaw_desired(1);
      acc_desired_(5) = yaw_desired(2);
      control_wrench_ = lqr_.action(x_, x_desired_, acc_desired_);
      this->publish_control_wrench();
      this->request_pwm_srv();
    }
  }
}
void Controller::publish_control_wrench()
{
  auto tau = geometry_msgs::msg::WrenchStamped();
  // The produced forces from the LQR controller are in NED frame.
  tau.header.frame_id = "swift/base_link_ned";
  tau.wrench.force.x = control_wrench_(0);
  tau.wrench.force.y = control_wrench_(1);
  tau.wrench.force.z = control_wrench_(2);
  tau.wrench.torque.x = control_wrench_(3);
  tau.wrench.torque.y = control_wrench_(4);
  tau.wrench.torque.z = control_wrench_(5);
  tau.header.stamp = this->get_clock()->now();
  tau_pub_->publish(tau);
}

void Controller::request_pwm_srv()
{
  Vector6d control_pwm = this->thrust_to_pwm();
  auto request = std::make_shared<custom_ros_interfaces::srv::PWM::Request>();
  request->x = control_pwm(0);
  request->y = control_pwm(1);
  request->z = control_pwm(2);
  request->roll = control_pwm(3);
  request->pitch = control_pwm(4);
  request->yaw = control_pwm(5);
  auto result = pwm_client_->async_send_request(request);
}
Vector6d Controller::thrust_to_pwm()
{
  /* There are 3 regions of thrust: deadband, +ve polynomial and -v polynomail
   * A 3rd order polynomial is fitted for the +/e regions using the data from
   * T-200 public performance data at 20V, y = f(x), where y is the PWM  and x
   * is the thrust force
   */
  Vector6d control_pwm;
  // Looping through 6D  wrench vector
  for (int i = 0; i <= 5; i++) {
    if (abs(control_wrench_(i)) < 0.5) {
      // The deadband region where we output a neutral PWM
      // 0.5 N is the lowest thrust
      control_pwm(i) = 1500;
    } else if (control_wrench_(i) > 0.5 && control_wrench_(i) <= 40) {
      // The +ve 3rd Polynomial region
      control_pwm(i) =
        1.52955373e+03 + 1.22981837e+01 * control_wrench_(i) +
        -1.92724532e-01 * control_wrench_(i) * control_wrench_(i) +
        1.39413493e-03 * control_wrench_(i) * control_wrench_(i) *
        control_wrench_(i);
    } else if (control_wrench_(i) < 0 && control_wrench_(i) >= -40) {
      // The -ve 3rd Polynomial region
      control_pwm(i) =
        1.47123141e+03 + 1.60739949e+01 * control_wrench_(i) +
        3.25097839e-01 * control_wrench_(i) * control_wrench_(i) +
        3.30373158e-03 * control_wrench_(i) * control_wrench_(i) *
        control_wrench_(i);
    }
    // Saturating the output PWM @ max and min PWM
    if (control_pwm(i) > 1890) {
      control_pwm(i) = 1890;
    }
    if (control_pwm(i) < 1120) {
      control_pwm(i) = 1120;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"controller_node"});
  std::shared_ptr<Controller> node = std::make_shared<Controller>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
