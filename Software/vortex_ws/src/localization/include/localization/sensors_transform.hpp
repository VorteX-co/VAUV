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
#ifndef LOCALIZATION__SENSORS_TRANSFORM_HPP_
#define LOCALIZATION__SENSORS_TRANSFORM_HPP_
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Transformer : public rclcpp::Node
{
public:
  Transformer();
  // Sensors callbacks
  void dvlCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

private:
  // Subscribers
  rclcpp::Subscription<
    geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr
    pressure_sub_;
  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;
  // TF listener
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  // TF buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // Sensors pose relative to the base_link in ENU frame
  Eigen::Quaterniond imu_rotation_;
  Eigen::Vector3d imu_translation_;
  Eigen::Quaterniond dvl_rotation_;
  Eigen::Vector3d dvl_translation_;
  Eigen::Vector3d pressure_translation_;
  // Transformed base_link velocities
  Eigen::Vector3d body_linear_velocity_;
  Eigen::Vector3d body_angular_velocity_;
};
#endif  // LOCALIZATION__SENSORS_TRANSFORM_HPP_
