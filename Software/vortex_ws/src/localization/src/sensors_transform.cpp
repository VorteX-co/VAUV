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
#include "sensors_transform.hpp"
#include <iostream>
#include <memory>

using std::placeholders::_1;

Transformer::Transformer()
: Node("sensors_transform")
{
  // Subscribers initialization
  dvl_sub_ =
    this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/swift/dvl_twist", rclcpp::SensorDataQoS(),
    std::bind(&Transformer::dvlCallback, this, _1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/swift/imu", rclcpp::SensorDataQoS(),
    std::bind(&Transformer::imuCallback, this, _1));
  pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "/swift/pressure", rclcpp::SensorDataQoS(),
    std::bind(&Transformer::pressureCallback, this, _1));
  // Publishers initialization
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/swift/odom",
      rclcpp::QoS(10));
  // TF2 buffer stores known frames and offers services for the relationships
  // between frames
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // Transform listener to request and receive coordinate frame transform
  // information over a tf2 buffer
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  try {
    // Look up for the transformation between a source frame and target frame
    // tf2::TimePointZero means the latest available transform in the buffer
    // 15 seconds timeout parameter, waiting for 15s until a transform becomes
    // available
    geometry_msgs::msg::TransformStamped dvl_transform;
    dvl_transform = tf_buffer_->lookupTransform(
      "swift/base_link", "swift/dvl_link", tf2::TimePointZero,
      tf2::durationFromSec(15));
    // Extracting  sensor cordinate translation and rotation in quaternion
    tf2::fromMsg(dvl_transform.transform.rotation, dvl_rotation_);
    tf2::fromMsg(dvl_transform.transform.translation, dvl_translation_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "Could not transform dvl_link static frame: %s",
      ex.what());
  }
  try {
    // Look up for the transformation between base_link and imu_link
    geometry_msgs::msg::TransformStamped imu_transform;
    imu_transform = tf_buffer_->lookupTransform(
      "swift/base_link", "swift/imu_link", tf2::TimePointZero,
      tf2::durationFromSec(15));
    tf2::fromMsg(imu_transform.transform.rotation, imu_rotation_);
    tf2::fromMsg(imu_transform.transform.translation, imu_translation_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "Could not transform imu_link static frame: %s",
      ex.what());
  }
  try {
    // Look up for the transformation between base_link and pressure_link
    geometry_msgs::msg::TransformStamped pressure_transform;
    pressure_transform = tf_buffer_->lookupTransform(
      "swift/base_link", "swift/pressure_link", tf2::TimePointZero,
      tf2::durationFromSec(15));
    tf2::fromMsg(pressure_transform.transform.translation,
      pressure_translation_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "Could not transform pressure_link static frame: %s",
      ex.what());
  }
  // The published Odometry msg
  odom_ = nav_msgs::msg::Odometry();
  odom_.header.frame_id = "swift/world_enu";
  odom_.child_frame_id = "swift/base_link";
}

void Transformer::dvlCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Velocity vector in the dvl frame
  Eigen::Vector3d v_dvl;
  tf2::fromMsg(msg->twist.twist.linear, v_dvl);
  // Rotation of the velocity vector v_dvl from dvl frame to body frame
  body_linear_velocity_ = dvl_rotation_.toRotationMatrix() * v_dvl;
  // DVL translation compensation
  body_linear_velocity_ += body_angular_velocity_.cross(dvl_translation_);
  // Publish the odometry msg to robot_localization node
  odom_.header.stamp = msg->header.stamp;
   odom_.twist.twist.linear.x = body_linear_velocity_ (0);
   odom_.twist.twist.linear.y = body_linear_velocity_ (1);
   odom_.twist.twist.linear.z = body_linear_velocity_ (2);
  odom_.twist.covariance = msg->twist.covariance;
  odom_pub_->publish(odom_);
}
void Transformer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Angular velocity in imu frame
  Eigen::Vector3d w_imu;
  tf2::fromMsg(msg->angular_velocity, w_imu);
  // rotating the vector w_imu by imu_rotation_ quaternion
  body_angular_velocity_ = imu_rotation_.toRotationMatrix() * w_imu;
}
void Transformer::pressureCallback(
  const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
  // compute the gauge pressure in Pa, atmospheric pressure is 101.325 kPa
  float Pgauge = (msg->fluid_pressure) - (101.325 * 1e+3);
  // depth in meter
  float h = Pgauge / (1025 * 9.806);
  // update odom z position, ENU
  odom_.pose.pose.position.z = -1 * h - pressure_translation_(2);
  odom_.pose.covariance[14] = msg->variance * msg->variance;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Transformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
