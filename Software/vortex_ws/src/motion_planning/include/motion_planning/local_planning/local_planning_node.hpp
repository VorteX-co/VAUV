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

#ifndef MOTION_PLANNING__LOCAL_PLANNING__LOCAL_PLANNING_NODE_HPP_
#define MOTION_PLANNING__LOCAL_PLANNING__LOCAL_PLANNING_NODE_HPP_

#include <memory>
#include <vector>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "steering.hpp"
#include "optimal_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class LocalPlanner : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes all the subscribes and publishers
   * @param Node options contains node name and paramteres from .yaml file
   */
  explicit LocalPlanner(const rclcpp::NodeOptions & options);
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
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr local_plan_pub_;

  /**
   * @brief  publish_pwm
   * @param pwm vector of length equal to the number of thrusters
   */
  void publish_local_plan(
    const Vector12d state, const Vector12d desired_state,
    const Vector6d feedforward_acc);
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
  Steering los_;
  Trajectory trajectory_generator_;
  Vector12d state_;
  Vector12d desired_state_;
  bool controller_on_{false};
  bool los_on_{false};
  double trajectory_duration_{0.0};    // Duration of the generated <6>DOF trajectory
  double trajectory_start_stamp_{0.0};  // Start timestamp for the trajectory
};
#endif  // MOTION_PLANNING__LOCAL_PLANNING__LOCAL_PLANNING_NODE_HPP_
