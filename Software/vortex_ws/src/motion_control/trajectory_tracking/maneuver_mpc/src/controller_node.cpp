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
#include "../include/maneuver_mpc/controller_node.hpp"
#include <math.h>
#include <map>
#include <string>
#include <memory>
using namespace std::chrono_literals;
using std::placeholders::_1;
/* Constructor */
ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  RCLCPP_INFO(this->get_logger(), " Maneuver MPC initialized ");
  // Subscribers

  guidance_sub = this->create_subscription<custom_ros_interfaces::msg::MMPC>(
    "/guidance/MMPC", 1, std::bind(&ControllerNode::GuidanceCB, this, _1));

  // Publishers
  wrench_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/rexrov/thruster_manager/input_stamped", 1);
  // msg
  tau = geometry_msgs::msg::WrenchStamped();
  ////////////////////////////////////////////////////////////////////////////////////
  /// Loading node paramteres
  std::map<string, double> mpc_params;
  mpc_params["N"] = this->get_parameter("N").as_double();
  mpc_params["dt"] = this->get_parameter("dt").as_double();
  mpc_params["w_xe"] = this->get_parameter("w_xe").as_double();
  mpc_params["w_ye"] = this->get_parameter("we_ye").as_double();
  mpc_params["we_psie"] = this->get_parameter("we_psie").as_double();
  mpc_params["we_ue"] = this->get_parameter("we_ue").as_double();
  mpc_params["we_ve"] = this->get_parameter("we_ve").as_double();
  mpc_params["we_re"] = this->get_parameter("we_re").as_double();
  mpc_params["w_Fx"] = this->get_parameter("w_Fx").as_double();
  mpc_params["w_Fy"] = this->get_parameter("w_Fy").as_double();
  mpc_params["w_Mz"] = this->get_parameter("w_Mz").as_double();
  mpc_params["w_Fx_dot"] = this->get_parameter("w_Fx_dot").as_double();
  mpc_params["w_Fy_dot"] = this->get_parameter("w_Fy_dot").as_double();
  mpc_params["w_Mz_dot"] = this->get_parameter("w_Mz_dot").as_double();
  mpc_params["Mx"] = this->get_parameter("Mx").as_double();
  mpc_params["My"] = this->get_parameter("Mpsi").as_double();
  mpc_params["Mpsi"] = this->get_parameter("Mpsi").as_double();
  mpc_params["LDx"] = this->get_parameter("LDx").as_double();
  mpc_params["LDy"] = this->get_parameter("LDy").as_double();
  mpc_params["LDpsi"] = this->get_parameter("LDpsi").as_double();
  mpc_params["QDx"] = this->get_parameter("QDx").as_double();
  mpc_params["QDy"] = this->get_parameter("QDy").as_double();
  mpc_params["QDpsi"] = this->get_parameter("QDpsi").as_double();
  mpc_params["max_xy"] = this->get_parameter("max_xy").as_double();
  mpc_params["max_psi"] = this->get_parameter("max_psi").as_double();
  mpc_params["max_u"] = this->get_parameter("max_u").as_double();
  mpc_params["max_v"] = this->get_parameter("max_v").as_double();
  mpc_params["max_r"] = this->get_parameter("max_r").as_double();
  mpc_params["max_Fx"] = this->get_parameter("max_Fx").as_double();
  mpc_params["max_Fy"] = this->get_parameter("max_Fy").as_double();
  mpc_params["max_Mz"] = this->get_parameter("max_Mz").as_double();
  xy_mpc.set_params(mpc_params);
}

/* Guidance data callback */
void ControllerNode::GuidanceCB(
  const custom_ros_interfaces::msg::MMPC::SharedPtr msg)
{
  Eigen::VectorXd state(12);
  Eigen::VectorXd command(6);
  Eigen::Vector3d feedforward;
  ////////////////////////////////////////////////////////////
  state << msg->x, msg->y, msg->psi, msg->u, msg->v, msg->r, msg->xe, msg->ye,
    msg->psie, msg->ue, msg->ve, msg->re;
  command << msg->xd, msg->yd, msg->psid, msg->ud, msg->vd, msg->rd;
  feedforward << msg->axd, msg->ayd, msg->apsid;
  auto forces = xy_mpc.Solve(state, command, feedforward);
  tau.wrench.force.x = forces[0];
  tau.wrench.force.y = -forces[1];
  tau.wrench.torque.z = -forces[2];
  RCLCPP_INFO(this->get_logger(), " Forces %f  %f  %f", forces[0], -forces[1],
    -forces[2]);
  wrench_pub->publish(tau);
}

/* ROS SPIN*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"maneuver_mpc"});
  auto node = std::make_shared<ControllerNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
