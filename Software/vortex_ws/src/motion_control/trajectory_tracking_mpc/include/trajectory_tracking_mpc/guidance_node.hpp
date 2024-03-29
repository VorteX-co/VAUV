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
#ifndef TRAJECTORY_TRACKING_MPC__GUIDANCE_NODE_HPP_
#define TRAJECTORY_TRACKING_MPC__GUIDANCE_NODE_HPP_
#include <Eigen/Core>
#include <memory>
#include "auv_core_headers.hpp"
#include "auv_structs.hpp"
#include "basic_trajectory.hpp"
#include "custom_ros_interfaces/msg/dmpc.hpp"
#include "custom_ros_interfaces/msg/mmpc.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "los_steering.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
/**
 * @brief The GuidanceNode class is the ros wrapper for the trajectory
 * generation and steering law for the trajectory tracking MPC
 */

class GuidanceNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes all the subscribes and publishers
   * within the node
   */
  explicit GuidanceNode(const rclcpp::NodeOptions & options);
  /**
   * @brief  generates a trajectory to the goal-waypoint
   * @param msg 3D waypoint
   */
  void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  /**
   * @brief stateCallback updates the local variables
   * @param msg auv-state
   */
  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  /**
   * @brief Publish function called by a timer for pulishing the reference
   * states for the MPC
   */
  void Publish();

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr waypoint_sub;
  // Publishers
  rclcpp::Publisher<custom_ros_interfaces::msg::MMPC>::SharedPtr MMPC_pub;
  rclcpp::Publisher<custom_ros_interfaces::msg::DMPC>::SharedPtr DMPC_pub;
  rclcpp::TimerBase::SharedPtr pubTimer;

  // Variables
  double t{0.0};  // clock for evaluating the generated trajectory
  double trajectoryDuration{0.0};
  Eigen::Vector3d goal{0.0, 0.0, 0.0};
  Eigen::Vector3d prev_goal{0.0, 0.0, 0.0};
  LOS los;  // for steering law
  auv_guidance::BasicTrajectory * basicTrajectory;
  auv_core::Vector13d trajectoryState;  // desired trajectory
  auv_core::Vector6d trajectoryAcc;     // desired trajectory acceleration

  // Messages
  custom_ros_interfaces::msg::MMPC MMPC_data;
  custom_ros_interfaces::msg::DMPC DMPC_data;
};

#endif  // TRAJECTORY_TRACKING_MPC__GUIDANCE_NODE_HPP_
