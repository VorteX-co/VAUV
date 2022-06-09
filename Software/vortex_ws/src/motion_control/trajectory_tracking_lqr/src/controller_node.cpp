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

#include "controller_node.hpp"
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

/* Constructor */
// =========================================================================
Controller::Controller(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  // Subscribers initialization
  local_plan_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/local_planning/plan", 10, std::bind(&Controller::feedbackCallback, this, _1));

  // Publishers initialization
  tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/swift/thruster_manager/input_stamped", 1);
  pwm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
    "/swift/thruster_manager/pwm", 1);
  /*****************************************************
   *  AUV parameters
   * ***************************************************/
  double mass = this->get_parameter("mass").as_double();
  double volume = this->get_parameter("volume").as_double();
  Eigen::VectorXd Ib =
    vector_to_eigen(this->get_parameter("Ib").as_double_array());
  Eigen::VectorXd Dlinear =
    vector_to_eigen(this->get_parameter("Dlinear").as_double_array());
  Eigen::VectorXd Dquad =
    vector_to_eigen(this->get_parameter("Dquad").as_double_array());
  Eigen::VectorXd Ma =
    vector_to_eigen(this->get_parameter("Ma").as_double_array());
  Eigen::VectorXd r_cog =
    vector_to_eigen(this->get_parameter("r_cog").as_double_array());
  Eigen::VectorXd r_cob =
    vector_to_eigen(this->get_parameter("r_cob").as_double_array());
  /*****************************************************
   * Controller parameters
   * ***************************************************/
  Eigen::VectorXd Q =
    vector_to_eigen(this->get_parameter("Q").as_double_array());
  Eigen::VectorXd R1 =
    vector_to_eigen(this->get_parameter("R").as_double_array());
  Eigen::VectorXd tau_max =
    vector_to_eigen(this->get_parameter("tau_max").as_double_array());
  Eigen::VectorXd error_max =
    vector_to_eigen(this->get_parameter("error_max").as_double_array());
  // Setting the Controller parameters
  this->lqr_.set_params(
    mass, volume, Ib, r_cob, r_cog, Ma, Dlinear, Dquad, Q,
    R1, tau_max, error_max);
  /*****************************************************
   *  Thrust allocation parameters
   * ***************************************************/
  // Number of thusters
  const int r = this->get_parameter("r").as_int();
  MatrixXd Tpose(6, r);
  for (int i = 0; i < r; i++) {
    // Getting thruster i pose paramter [T1 .. Tr]
    std::string Tindex = std::to_string(i + 1);
    Tpose.col(i) =
      vector_to_eigen(this->get_parameter("T" + Tindex).as_double_array());
  }
  const int k = this->get_parameter("k").as_double();
  VectorXd coeff_left = vector_to_eigen(
    this->get_parameter("rpm_to_pwm_coeff_left").as_double_array());
  VectorXd coeff_right = vector_to_eigen(
    this->get_parameter("rpm_to_pwm_coeff_right").as_double_array());
  allocator_.set_params(k, Tpose, coeff_left, coeff_right);
}
// =========================================================================
/* Callbacks*/
void Controller::feedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // Converting the Float32MultiArray msg to Eigen::VectorXd
  std::vector<double> plan_vector(msg->data.begin(), msg->data.end());
  VectorXd plan = vector_to_eigen(plan_vector);
  Vector12d state = plan.head<12>();
  Vector12d desired_state = plan.segment<12>(12);
  Vector6d desired_acc = plan.tail<6>();
  // Taking control action
  Vector6d wrench = lqr_.action(state, desired_state, desired_acc);
  // Publishing the output
  this->publish_control_wrench(wrench);
  Eigen::VectorXd pwm = allocator_.wrench_to_pwm_thrusters(wrench);
  this->publish_pwm(pwm);
}
// =========================================================================
void Controller::publish_control_wrench(const Eigen::VectorXd & wrench)
{
  auto tau = geometry_msgs::msg::WrenchStamped();
  // The produced forces from the controller are in NED frame.
  tau.header.frame_id = "swift/base_link_ned";
  tau.wrench.force.x = wrench(0);
  tau.wrench.force.y = wrench(1);
  tau.wrench.force.z = wrench(2);
  tau.wrench.torque.x = wrench(3);
  tau.wrench.torque.y = wrench(4);
  tau.wrench.torque.z = wrench(5);
  tau.header.stamp = this->get_clock()->now();
  tau_pub_->publish(tau);
}
// =========================================================================
void Controller::publish_pwm(const Eigen::VectorXd & pwm)
{
  std::vector<int> pwm_vec(pwm.data(), pwm.data() + pwm.rows() * pwm.cols());
  auto pwm_msg = std_msgs::msg::Int32MultiArray();
  pwm_msg.data = pwm_vec;
  this->pwm_pub_->publish(pwm_msg);
}
// =========================================================================
inline Eigen::VectorXd Controller::vector_to_eigen(std::vector<double> v)
{
  // A method for converting std::vector<double> to Eigen::VectorXd
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}
// =========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"controller_node"});
  std::shared_ptr<Controller> node = std::make_shared<Controller>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
