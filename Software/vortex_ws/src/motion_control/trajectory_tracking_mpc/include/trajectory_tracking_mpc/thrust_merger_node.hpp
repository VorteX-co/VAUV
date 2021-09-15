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
#ifndef TRAJECTORY_TRACKING_MPC__THRUST_MERGER_NODE_HPP_
#define TRAJECTORY_TRACKING_MPC__THRUST_MERGER_NODE_HPP_
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"

class ThrustMergerNode : public rclcpp::Node
{
public:
  // Constructor
  explicit ThrustMergerNode(const rclcpp::NodeOptions & options);
  // callbacks
  void ManeuverCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void DepthCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void TimerCB();

private:
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    maneuver_sub;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr depth_sub;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;

  // Variables
  double Fx{0.0};
  double Fy{0.0};
  double Mz{0.0};
  double Fz{0.0};
  rclcpp::TimerBase::SharedPtr pubTimer;

  // Messages
  geometry_msgs::msg::WrenchStamped merged_tau;
};
#endif  // TRAJECTORY_TRACKING_MPC__THRUST_MERGER_NODE_HPP_
