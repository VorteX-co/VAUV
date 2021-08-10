// Copyright 2021 VorteX-co
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
/* Include guard to prevent double declaration of identifiers
   such as types, enums and static variacles */
#ifndef MANEUVER_MPC__CONTROLLER_NODE_HPP_
#define MANEUVER_MPC__CONTROLLER_NODE_HPP_
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "custom_ros_interfaces/msg/mmpc.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "maneuver_mpc.hpp"
#include "rclcpp/rclcpp.hpp"
/**
 * @brief The ControllerNode class is a ros wrapper for the maneuver MPC class
 */

class ControllerNode : public rclcpp::Node
{
public:
  // Constructor
  explicit ControllerNode(const rclcpp::NodeOptions & options);
  // callbacks
  void GuidanceCB(const custom_ros_interfaces::msg::MMPC::SharedPtr msg);

private:
  // Subscribers
  rclcpp::Subscription<custom_ros_interfaces::msg::MMPC>::SharedPtr
    guidance_sub;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;

  // Variables
  Maneuver::MPC xy_mpc;

  // Messages
  geometry_msgs::msg::WrenchStamped tau;
};
#endif  // MANEUVER_MPC__CONTROLLER_NODE_HPP_
