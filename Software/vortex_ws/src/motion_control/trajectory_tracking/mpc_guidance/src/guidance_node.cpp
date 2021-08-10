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
#include "../include/mpc_guidance/guidance_node.hpp"
#include <math.h>
#include <functional>
#include <iostream>
#include <memory>
#include "../lib/auv_guidance/include/waypoint.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* Constructor */
GuidanceNode::GuidanceNode(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options)
{
  waypoint_sub = this->create_subscription<geometry_msgs::msg::Point>(
    "/guidance/waypoint", 1,
    std::bind(&GuidanceNode::pointCallback, this, _1));
  // Publishers for both maneuver and depth MPCs
  MMPC_pub = this->create_publisher<custom_ros_interfaces::msg::MMPC>(
    "/guidance/MMPC", 1);
  MMPC_data = custom_ros_interfaces::msg::MMPC();
  DMPC_pub = this->create_publisher<custom_ros_interfaces::msg::DMPC>(
    "/guidance/DMPC", 1);
  DMPC_data = custom_ros_interfaces::msg::DMPC();
}

/* Callbacks*/
void GuidanceNode::pointCallback(
  const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Constraints are for trajectory limits
  auv_core::auvConstraints * auvConstraints_;
  auv_guidance::Waypoint * startWaypoint_, * endWaypoint_;
  auvConstraints_ = new auv_core::auvConstraints;
  auvConstraints_->maxXYDistance = this->get_parameter("max_xy").as_double();
  auvConstraints_->maxZDistance = this->get_parameter("max_z").as_double();
  auvConstraints_->maxAlignInclination =
    this->get_parameter("max_align_inclination").as_double();
  auvConstraints_->maxTransVel(0) =
    this->get_parameter("max_x_vel").as_double();
  auvConstraints_->maxTransVel(1) =
    this->get_parameter("max_y_vel").as_double();
  auvConstraints_->maxTransVel(2) =
    this->get_parameter("max_z_vel").as_double();
  auvConstraints_->maxRotVel = this->get_parameter("max_rot_vel").as_double();
  auvConstraints_->maxTransAccel(0) =
    this->get_parameter("max_x_acc").as_double();
  auvConstraints_->maxTransAccel(1) =
    this->get_parameter("max_y_acc").as_double();
  auvConstraints_->maxTransAccel(2) =
    this->get_parameter("max_z_acc").as_double();
  auvConstraints_->maxRotAccel = this->get_parameter("max_rot_acc").as_double();
  auvConstraints_->transJerk =
    this->get_parameter("max_trans_jerk").as_double();
  auvConstraints_->rotJerk = this->get_parameter("max_rot_jerk").as_double();

  // Start state
  // Inertial Position, velocity, and acceleration expressed in I-frame
  Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
  Eigen::Vector3d posIStart = zero3d;
  posIStart << MMPC_data.x, MMPC_data.y, DMPC_data.z;
  Eigen::Vector3d velIStart = zero3d;
  velIStart << MMPC_data.u, MMPC_data.v, DMPC_data.w;
  Eigen::Vector3d accelIStart = zero3d;
  Eigen::Quaterniond quaternion_;  // Attitude wrt I-frame
  Eigen::Vector3d angVelB_;        // Angular velocity about B-frame axis
  startWaypoint_ = new auv_guidance::Waypoint(posIStart, velIStart, accelIStart,
      quaternion_, angVelB_);
  // End state
  Eigen::Vector3d posIEnd = zero3d;
  posIEnd << msg->x, msg->y, msg->z;
  // We are not using the attitude trajectory generation, so quatEnd is left
  // zero We are using LOS for generating the desired yaw angle, pitch and roll
  // are assumed to be zero all the time
  Eigen::Quaterniond quatEnd;
  endWaypoint_ =
    new auv_guidance::Waypoint(posIEnd, zero3d, zero3d, quatEnd, zero3d);
  // Starting the trajectory generation
  basicTrajectory = new auv_guidance::BasicTrajectory(
    auvConstraints_, startWaypoint_, endWaypoint_);
  trajectoryDuration = basicTrajectory->getDuration();
  // Line of sight initialization
  Eigen::Vector2d setpoint{msg->x, msg->y};
  los.setpoint(setpoint);
  t = 12.0;
  state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/rexrov/pose_gt", 1, std::bind(&GuidanceNode::stateCallback, this, _1));
  // Starting to publish the reference states for the MPCs
  pubTimer =
    this->create_wall_timer(50ms, std::bind(&GuidanceNode::Publish, this));
}
void GuidanceNode::stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  MMPC_data.x = msg->pose.pose.position.x;
  MMPC_data.y = msg->pose.pose.position.y;
  DMPC_data.z = msg->pose.pose.position.z;
  double q0 = msg->pose.pose.orientation.x;
  double q1 = msg->pose.pose.orientation.y;
  double q2 = msg->pose.pose.orientation.z;
  double q3 = msg->pose.pose.orientation.w;
  MMPC_data.psi =
    std::atan2(2 * (q3 * q2 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2));
  DMPC_data.theta = std::asin(2 * (q3 * q1 - q0 * q2));
  MMPC_data.u = msg->twist.twist.linear.x;
  MMPC_data.v = msg->twist.twist.linear.y;
  DMPC_data.w = msg->twist.twist.linear.z;
  MMPC_data.r = msg->twist.twist.angular.z;
  DMPC_data.q = msg->twist.twist.angular.y;
}
void GuidanceNode::Publish()
{
  // Evaluating the generated trajectory
  trajectoryState = basicTrajectory->computeState(t);
  trajectoryAcc = basicTrajectory->computeAccel(t);
  t += 0.05;
  // LOS stuff
  Eigen::Vector3d los_pose{MMPC_data.x, MMPC_data.y, MMPC_data.psi};
  Eigen::Vector3d los_vel{MMPC_data.u, MMPC_data.v, MMPC_data.r};
  los.update_state(los_pose, los_vel);
  los.calculate_reference();
  ///////////////////////////////////////////////////////////////////////////////
  /// Desired trajectory states
  MMPC_data.xd = trajectoryState(0);
  MMPC_data.yd = trajectoryState(1);
  DMPC_data.zd = trajectoryState(2);
  MMPC_data.psid = los.yaw_des;
  MMPC_data.ud = trajectoryState(3);
  MMPC_data.vd = 0.0;
  DMPC_data.wd = trajectoryState(5);
  MMPC_data.rd = los.rd;
  MMPC_data.axd = trajectoryAcc(0);
  MMPC_data.ayd = 0.0;
  DMPC_data.azd = trajectoryAcc(2);
  MMPC_data.apsid = los.rdd;
  //////////////////////////////////////////////////////////////////////////////
  /// Error states
  MMPC_data.xe = (MMPC_data.x - MMPC_data.xd) * -cos(MMPC_data.psi) -
    (MMPC_data.y - MMPC_data.yd) * std::sin(MMPC_data.psi);
  MMPC_data.ye = (MMPC_data.x - MMPC_data.xd) * std::sin(MMPC_data.psi) -
    (MMPC_data.y - MMPC_data.yd) * cos(MMPC_data.psi);
  DMPC_data.ze = DMPC_data.z - DMPC_data.zd;
  MMPC_data.psie = MMPC_data.psid - MMPC_data.psi;
  MMPC_data.ue = MMPC_data.u - MMPC_data.ud * cos(MMPC_data.psie) +
    MMPC_data.vd * std::sin(MMPC_data.psie);
  MMPC_data.ve = MMPC_data.v - MMPC_data.ud * std::sin(MMPC_data.psie) -
    MMPC_data.vd * cos(MMPC_data.psie);
  MMPC_data.re = MMPC_data.r - MMPC_data.rd;
  DMPC_data.thetae = DMPC_data.theta + DMPC_data.q * 0.05;
  DMPC_data.we = DMPC_data.w - DMPC_data.wd;

  ///////////////////////////////////////////////////////////////////////////////////
  MMPC_pub->publish(MMPC_data);
  DMPC_pub->publish(DMPC_data);
}
/* ROS SPIN*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments({"mpc_guidance"});
  std::shared_ptr<GuidanceNode> node = std::make_shared<GuidanceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
