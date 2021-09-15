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
#include "depth_node.hpp"
#include <math.h>
#include <string>
#include <map>
#include <memory>
using namespace std::chrono_literals;
using std::placeholders::_1;
/* Constructor */
DepthNode::DepthNode(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  RCLCPP_INFO(this->get_logger(), " Depth MPC initialized ");
  // Subscribers
  rclcpp::QoS qos(10);
  guidance_sub = this->create_subscription<custom_ros_interfaces::msg::DMPC>(
    "/guidance/DMPC", 1, std::bind(&DepthNode::GuidanceCB, this, _1));
  // Publishers
  wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/depth/tau", qos);
  // msg
  tau = geometry_msgs::msg::WrenchStamped();
  ////////////////////////////////////////////////////////////////////////////////////
  /// Loading node paramteres
  std::map<string, double> mpc_params;
  mpc_params["N"] = this->get_parameter("N").as_double();
  mpc_params["dt"] = this->get_parameter("dt").as_double();
  mpc_params["w_ze"] = this->get_parameter("w_ze").as_double();
  mpc_params["w_thetae"] = this->get_parameter("w_thetae").as_double();
  mpc_params["w_we"] = this->get_parameter("w_we").as_double();
  mpc_params["w_Fz"] = this->get_parameter("w_Fz").as_double();
  mpc_params["w_My"] = this->get_parameter("w_My").as_double();
  mpc_params["w_Fz_dot"] = this->get_parameter("w_Fz_dot").as_double();
  mpc_params["w_My_dot"] = this->get_parameter("w_My_dot").as_double();
  mpc_params["Mz"] = this->get_parameter("Mz").as_double();
  mpc_params["Mtheta"] = this->get_parameter("Mtheta").as_double();
  mpc_params["LDz"] = this->get_parameter("LDz").as_double();
  mpc_params["LDtheta"] = this->get_parameter("LDtheta").as_double();
  mpc_params["QDz"] = this->get_parameter("QDz").as_double();
  mpc_params["QDtheta"] = this->get_parameter("QDtheta").as_double();
  mpc_params["max_z"] = this->get_parameter("max_z").as_double();
  mpc_params["max_theta"] = this->get_parameter("max_theta").as_double();
  mpc_params["max_w"] = this->get_parameter("max_w").as_double();
  mpc_params["max_Fz"] = this->get_parameter("max_Fz").as_double();
  mpc_params["max_My"] = this->get_parameter("max_My").as_double();
  mpc_params["weight"] = this->get_parameter("weight").as_double();
  mpc_params["buoyancy"] = this->get_parameter("buoyancy").as_double();
  z_mpc.set_params(mpc_params);
}

/* Guidance data callback */
void DepthNode::GuidanceCB(
  const custom_ros_interfaces::msg::DMPC::SharedPtr msg)
{
  Eigen::VectorXd state(7);
  Eigen::VectorXd command(2);
  double feedforward;
  ////////////////////////////////////////////////////////////
  state << msg->z, msg->theta, msg->w, msg->q, msg->ze, msg->thetae, msg->we;
  command << msg->zd, msg->wd;
  feedforward = msg->azd;
  auto forces = z_mpc.Solve(state, command, feedforward);
  tau.wrench.force.z = -forces[0];
  tau.wrench.torque.y = -forces[1];
  RCLCPP_INFO(this->get_logger(), " Depth MPC Forces %f  %f ", -forces[0],
    -forces[1]);
  rclcpp::Time now = this->get_clock()->now();
  tau.header.stamp = now;
  wrench_pub->publish(tau);
}

/* ROS SPIN*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"depth_node"});
  auto node = std::make_shared<DepthNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
