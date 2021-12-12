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
#ifndef CONTROL_ALLOCATION__ALLOCATION_NODE_HPP_
#define CONTROL_ALLOCATION__ALLOCATION_NODE_HPP_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Allocator : public rclcpp::Node
{
public:
  explicit Allocator(const rclcpp::NodeOptions & options);
  /**
   * @brief wrenchCallback handles control forces msg
   * @param msg WrenchStamped [force3,torque3]
   */
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

private:
  /**
   * @brief thrust_configuration compute thruster contribution for every DOF
   * @param Tpose thruster position and orientation w.r.t COG [x,y,z,r,p,y]
   * @return 6DOF thrust configuration vector
   */
  Vector6d thrust_configuration(const Vector6d & Tpose);
  /**
   * @brief  rpm_to_pwm compute commanded pwm for each thruster
   * @param n thruster rpm
   * @return pwm vector
   */
  Eigen::VectorXd rpm_to_pwm(Eigen::VectorXd & n);
  /**
   * @brief  publish_pwm
   * @param pwm vector of length equal to the number of thrusters
   */
  void publish_pwm(Eigen::VectorXd & pwm);

  // B† Moore–Penrose pseudo-inverse of B
  // B = T K     .. where T is the thruster configuration matrix
  //                                K is the thust coefficient matrix
  Eigen::MatrixXd Bpinv_;
  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
  // Publisher
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;
};

#endif  // CONTROL_ALLOCATION__ALLOCATION_NODE_HPP_
