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

#ifndef TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
#define TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
#include <memory>
#include <vector>
#include "custom_ros_interfaces/srv/pwm.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "los_steering.hpp"
#include "lqr.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "trajectory.hpp"

enum modes
{
  station_keeping = 0,
  point_tracking = 1,
  attitude_tracking = 2,
  roll_tracking = 3,
  pitch_tracking = 4,
  yaw_tracking = 5,
};

class Controller : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes all the subscribes and publishers
   * @param Node options contains node name and paramteres from .yaml file
   */
  explicit Controller(const rclcpp::NodeOptions & options);
  /**
   * @brief pointCallback handles commanded waypoint
   * @param msg Point [xd,yd,zd]
   */
  void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  /**
   * @brief attitudeCallback handles commanded 3D rotations
   * @param msg desired [rpy]
   */
  void attitudeCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  /**
   * @brief handle commanded roll angle
   * @param msg Float32 [φd]
   */
  void rollCallback(const std_msgs::msg::Float32::SharedPtr msg);
  /**
   * @brief handle commanded pitch angle
   * @param msg Float32 [θd]
   */
  void pitchCallback(const std_msgs::msg::Float32::SharedPtr msg);
  /**
   * @brief handle commanded yaw angle
   * @param msg Float32 [ψd]
   */
  void yawCallback(const std_msgs::msg::Float32::SharedPtr msg);
  /**
   * @brief handle hold command for pose keeping
   * @param msg Float64 [not used data]
   */
  void holdCallback(const std_msgs::msg::Float32::SharedPtr msg);
  /**
   * @brief handle AUV state
   * @param msg Odometry [pose η, velocity ν]
   */
  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cmd_waypoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cmd_attitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_roll_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_hold_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_;
  // Service client
  rclcpp::Client<custom_ros_interfaces::srv::PWM>::SharedPtr pwm_client_;

  /**
   * @brief Publish the computed control forces and moments
   */
  void publish_control_wrench();
  /**
   * @brief Request a control action from pix4
   */
  void request_pwm_srv();

  /**
   * @brief from quaternions to Euler conversion
   * @param quaternions vector
   */
  Vector3d q_to_euler(Vector4d quat);
  /**
   * @brief Convert the control forces to PWM for T200 thrusters
   */
  Vector6d thrust_to_pwm();
  /*
   * @brief Convert std::vector to Eigen::VectorXd
   * @param: std::vector<double> v
   */
  inline Eigen::VectorXd vector_to_eigen(std::vector<double> v);

  // Variables
  Vector12d x_;           // AUV state
  Vector12d x_desired_;   // Desired stated
  Vector12d x_hold_;      // Hold state
  Vector6d acc_desired_;  // Desired 6DOF acceleration
  TrajectoryGenerator trajectory_generator_;
  ILOS los_;                 //  Line of sight steering-law
  LQR lqr_;                  //  Linear Quadratic Regulator
  Vector6d control_wrench_;  //  Control forces and moments
  bool controller_on_{false};
  int control_mode_{0};  // Default station keeping
  double translation_duration_{
    0.0};    // Duration of the generated translation trajectory
  double translation_start_stamp_{0.0};  // Start timestamp for the translation
  double translation_clock_{
    0.0};    // Clock for evaluating the generated translation trajectory
  double rotation_duration_{
    0.0};    // Duration of the generated rotation trajectory
  double rotation_start_stamp_{0.0};  // Start timestamp for the rotation
  double rotation_clock_{
    0.0};    // Clock for evaluating the generated translation trajectory
};

#endif  // TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
