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
#include "thrust_merger_node.hpp"
#include <memory>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

/* Constructor */
ThrustMergerNode::ThrustMergerNode(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  RCLCPP_INFO(this->get_logger(), "Thrust merger node initialized ");

  maneuver_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/maneuver/tau", 1, std::bind(&ThrustMergerNode::ManeuverCB, this, _1));
  depth_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/depth/tau", 1, std::bind(&ThrustMergerNode::DepthCB, this, _1));

  // Publishers
  wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/rexrov/thruster_manager/input_stamped", 1);
  // msg
  merged_tau = geometry_msgs::msg::WrenchStamped();
  pubTimer = this->create_wall_timer(
    75ms, std::bind(&ThrustMergerNode::TimerCB, this));
}

void ThrustMergerNode::ManeuverCB(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  Fx = msg->wrench.force.x;
  Fy = msg->wrench.force.y;
  Mz = msg->wrench.torque.z;
}
void ThrustMergerNode::DepthCB(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  Fz = msg->wrench.force.z;
}
void ThrustMergerNode::TimerCB()
{
  merged_tau.wrench.force.x = Fx;
  merged_tau.wrench.force.y = Fy;
  merged_tau.wrench.torque.z = Mz;
  merged_tau.wrench.force.z = Fz;
  wrench_pub->publish(merged_tau);
}

/* ROS SPIN*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"thrust_merger_node"});
  auto node = std::make_shared<ThrustMergerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
