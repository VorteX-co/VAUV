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

#ifndef TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
#define TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_

#include <memory>
#include <vector>
#include "custom_ros_interfaces/srv/pwm.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "lqr.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "thrust_allocation.hpp"

class Controller : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes all the subscribes and publishers
   * @param Node options contains node name and paramteres from .yaml file
   */
  explicit Controller(const rclcpp::NodeOptions & options);
  /**
   * @brief feedbackCallback handles local planning node outputs
   * @param msg Float32MultiArray [state, desired_state]
   */
  void feedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr local_plan_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;

  /**
   * @brief Publish the computed control forces and moments
   * @param VectorXd [control forces and moments]
   */
  void publish_control_wrench(const Eigen::VectorXd & wrench);
  /**
   * @brief  publish_pwm
   * @param pwm vector of length equal to the number of thrusters
   */
  void publish_pwm(const Eigen::VectorXd & pwm);
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

  // Linear Quadratic Regulator
  LQR lqr_;
  // Class for allocating the generalized control forces to each thruster
  Allocator allocator_;
};
#endif  // TRAJECTORY_TRACKING_LQR__CONTROLLER_NODE_HPP_
