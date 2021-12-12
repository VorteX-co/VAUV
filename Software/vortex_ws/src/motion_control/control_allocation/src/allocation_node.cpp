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

#include "allocation_node.hpp"
#include <math.h>
#include <tf2_eigen/tf2_eigen.h>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

/* Constructor */
Allocator::Allocator(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  // =========================================================================
  // Subscriber intilization
  rclcpp::Parameter wrench_topic_param =
    this->get_parameter("control_wrench_topic");
  std::string wrench_topic_name = wrench_topic_param.as_string();
  wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    wrench_topic_name, 1, std::bind(&Allocator::wrenchCallback, this, _1));
  // =========================================================================
  // Publisher intilization
  rclcpp::Parameter pwm_topic_param = this->get_parameter("output_pwm_topic");
  std::string pwm_topic_name = pwm_topic_param.as_string();
  pwm_pub_ =
    this->create_publisher<std_msgs::msg::Int32MultiArray>(pwm_topic_name, 1);
  // =========================================================================
  // Number of thusters
  rclcpp::Parameter r_param = this->get_parameter("r");
  const int r = r_param.as_int();
  // Number of control forces
  rclcpp::Parameter n_param = this->get_parameter("n");
  const int n = n_param.as_int();
  // Thruster configuration matrix
  Eigen::MatrixXd Tconfig(n, r);

  for (int i = 0; i < r; i++) {
    // Getting thruster i pose paramter [T1 .. Tr]
    std::string Tindex = std::to_string(i + 1);
    rclcpp::Parameter Ti_param = this->get_parameter("T" + Tindex);
    std::vector<double> Ti_array = Ti_param.as_double_array();
    // Converting to Eigen vector
    Eigen::VectorXd Ti = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      Ti_array.data(), Ti_array.size());
    // Calculating the Thrust Configuration of thruster i
    Tconfig.col(i) = this->thrust_configuration(Ti);
  }
  // =========================================================================
  // Force Coefficient parameter ( N / rpm )
  rclcpp::Parameter k_param = this->get_parameter("k");
  const double k = k_param.as_double();
  // Force Coefficient Matrix for the r thrusters
  Eigen::MatrixXd K = k * Eigen::MatrixXd::Identity(r, r);
  // =========================================================================
  // τ = T K u    -->   τ = B u    ..   B(n,r)
  Eigen::MatrixXd B = Tconfig * K;
  // u = B† τ    --> B† = Bpinv(r,n)  Moore–Penrose inverse of B
  this->Bpinv_ = B.transpose() * (B * B.transpose()).inverse();
}
// -------------------------------------------------------------------------------------------------------------------------------------------
/* Callbacks*/
void Allocator::wrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  /* Given control forces and torques τ
   * Compute thrusters allocated control input u
   * u = B† τ
   * reference: equation (12.254) Fossen 2011
   */
  // Extracting control froces and torques
  Eigen::Matrix<double, 3, 1> force;
  Eigen::Matrix<double, 3, 1> torque;
  tf2::fromMsg(msg->wrench.force, force);
  tf2::fromMsg(msg->wrench.torque, torque);
  Eigen::VectorXd tau(force.size() + torque.size());
  // The generalized control vector
  tau << force, torque;
  // Number of thrusters for allocation r
  rclcpp::Parameter r_param = this->get_parameter("r");
  const int r = r_param.as_int();
  Eigen::VectorXd u_alloc(r);
  //  u = B† τ
  u_alloc = this->Bpinv_ * tau;
  // Commanded r propellers speed vector n (RPM)
  Eigen::VectorXd n(r);
  for (size_t i = 0; i < r; i++) {
    // u = n |n|
    // n = sign(u) * sqrt(|u|)
    n(i) = (u_alloc(i) < 0) ?
      -1 :
      (u_alloc(i) > 0) * std::sqrt(std::abs(u_alloc(i)));
  }
  Eigen::VectorXd pwm = this->rpm_to_pwm(n);
  this->publish_pwm(pwm);
}
// -------------------------------------------------------------------------------------------------------------------------------------------
void Allocator::publish_pwm(Eigen::VectorXd & pwm)
{
  std::vector<int> pwm_vec(pwm.data(), pwm.data() + pwm.rows() * pwm.cols());
  auto pwm_msg = std_msgs::msg::Int32MultiArray();
  pwm_msg.data = pwm_vec;
  this->pwm_pub_->publish(pwm_msg);
}
Vector6d Allocator::thrust_configuration(const Vector6d & Tpose)
{
  // Given  Thruster pose [X,Y,Z,φ,θ,ψ]
  /* Computer the contribution to every DOF
   *
   *           Surge                  cos θ cos φ
   *           Sway                   sin θ cos φ
   *           Heave                      sin φ
   *           Roll              =    −Z sin θ + Y sin φ
   * Ti =    Pitch                  −Z cos θ + X sin φ
   *           Yaw                   −Y cos θ + X sin θ
   * Reference: Viktor Berg (2012) equation(2.38)
   */
  double stheta = sin(Tpose(4));
  double ctheta = cos(Tpose(4));
  double spsi = sin(Tpose(5));
  double cpsi = cos(Tpose(5));
  Vector6d T;
  T << ctheta * cpsi, ctheta * spsi, stheta,
    Tpose(1) * stheta - Tpose(2) * spsi, Tpose(0) * stheta - Tpose(2) * cpsi,
    Tpose(0) * spsi - Tpose(1) * cpsi;
  return T;
}
// -------------------------------------------------------------------------------------------------------------------------------------------
Eigen::VectorXd Allocator::rpm_to_pwm(Eigen::VectorXd & n)
{
  /* Given n vector (rpm) compute the equivalent pwm
   * There are 3 operation regions [left,neutral,right]
   * [left,right] for -ve and +ve rpm respectively
   * The relationship between [pwm and rpm] for each region
   *  is approximated to 3rd polynomial ,, y = a + b * x + c * x^2 + d * x^3
   * x is the rpm value and y is the output pwm value
   * The coefficients [a,b,c,d] are computed by fitting 3rd polynomial between
   *  Thw PWM to RPM values
   * The coefficients are calculated offline and passed as parameter vector
   */
  rclcpp::Parameter neutral_pwm_param = this->get_parameter("neutral_pwm");
  const int neutral_pwm = neutral_pwm_param.as_int();
  rclcpp::Parameter min_rpm_param = this->get_parameter("min_rpm");
  const int min_rpm = min_rpm_param.as_int();
  rclcpp::Parameter r_param = this->get_parameter("r");
  const int r = r_param.as_int();
  Eigen::VectorXd pwm(r);
  for (int i = 0; i < r; i++) {
    if (abs(n(i)) < min_rpm) {
      pwm(i) = neutral_pwm;
    } else if (abs(n(i)) > min_rpm && n(i) > 0) {
      rclcpp::Parameter coeff_right_param =
        this->get_parameter("rpm_to_pwm_coeff_right");
      std::vector<double> coeff_right = coeff_right_param.as_double_array();
      pwm(i) = coeff_right[3] + coeff_right[2] * n(i) +
        coeff_right[1] * n(i) * n(i) +
        coeff_right[0] * n(i) * n(i) * n(i);
    } else if (abs(n(i)) > min_rpm && n(i) < 0) {
      rclcpp::Parameter coeff_left_param =
        this->get_parameter("rpm_to_pwm_coeff_left");
      std::vector<double> coeff_left = coeff_left_param.as_double_array();
      pwm(i) = coeff_left[3] + coeff_left[2] * abs(n(i)) +
        coeff_left[1] * abs(n(i)) * abs(n(i)) +
        coeff_left[0] * abs(n(i)) * abs(n(i)) * abs(n(i));
    }

    rclcpp::Parameter max_pwm_param = this->get_parameter("max_pwm");
    const int max_pwm = max_pwm_param.as_int();
    if (pwm(i) > max_pwm) {
      pwm(i) = max_pwm;
    }
    rclcpp::Parameter min_pwm_param = this->get_parameter("min_pwm");
    const int min_pwm = min_pwm_param.as_int();
    if (pwm(i) < min_pwm) {
      pwm(i) = min_pwm;
    }
  }
  return pwm;
}
// -------------------------------------------------------------------------------------------------------------------------------------------
/* ROS SPIN*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"allocation_node"});
  std::shared_ptr<Allocator> node = std::make_shared<Allocator>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
