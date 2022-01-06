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

#ifndef MOTION_CONTROL__CONTROLLER_NODE_HPP_
#define MOTION_CONTROL__CONTROLLER_NODE_HPP_
#include <memory>
#include <vector>
#include "custom_ros_interfaces/srv/pwm.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "guidance.hpp"
#include "mpc.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

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
  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cmd_waypoint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cmd_attitude_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_;

  /**
   * @brief Publish the computed control forces and moments
   */
  void publish_control_wrench();
  /**
   * @brief from quaternions to Euler conversion
   * @param quaternions vector
   */
  Vector3d quaternion_to_euler(const Vector4d & quaternion);
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
  Guidance guidance_;
  MPC mpc_;                  //  Linear Quadratic Regulator
  Vector6d control_wrench_;  //  Control forces and moments
  bool controller_on_{false};
  int control_mode_{0};  // Default station keeping
  double trajectory_duration_{
    0.0};    // Duration of the generated <6>DOF trajectory
  double trajectory_start_stamp_{0.0};  // Start timestamp for the trajectory
};

#endif  // MOTION_CONTROL__CONTROLLER_NODE_HPP_
