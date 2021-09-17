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
#include "../../../include/VDVL/serial_node.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include "../../../include/VDVL/serial_port.hpp"
#include "custom_ros_interfaces/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;
VDVL::SerialNode::SerialNode()
: Node("serial_node")
{
  RCLCPP_INFO(this->get_logger(), " VDVL initialized :) ");
  this->declare_parameter<std::string>("portName", "/dev/ttyUSB0");
  this->get_parameter("portName", _portName);
  this->declare_parameter<int64_t>("baudRate", 115200);
  this->get_parameter("baudRate", _baudRate);
  std::string topic = "Vortex/DVL";
  _publisher =
    this->create_publisher<custom_ros_interfaces::msg::DVL>(topic, 10);
}
void VDVL::SerialNode::Create(boost::asio::io_service & ios)
{
  /* *******************************************************
   * Method for creating a SerialPort instant
   * openning the serial Port
   * setting the onRead callback
   * ******************************************************* */
  try {
    _serial.reset(new VDVL::SerialPort(ios, _portName));
    _serial->Open(boost::bind(&VDVL::SerialNode::OnRead, this, _1, _2));
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
  }
}
void VDVL::SerialNode::OnPublish(std::string buffer)
{
  /* *************************************************
   * Publishing callback
   * ************************************************/
  RCLCPP_INFO(this->get_logger(), "On Publishing");
  std::vector<std::string> parts;
  boost::split(parts, buffer, boost::is_any_of(","));
  size_t n = parts.size();
  if (n == 9 && parts[0] == "wrx") {
    auto msg = custom_ros_interfaces::msg::DVL();
    // rclcpp clock
    msg.header.stamp = this->get_clock()->now();
    // Frame ID for later data transformation
    msg.header.frame_id = "swift/dvl_link";
    // time passed since last velocity report (ms) / 1000
    msg.dt = ::atof(parts[1].c_str()) / 1000;
    // the velocity report
    msg.twist.linear.x = ::atof(parts[2].c_str());
    msg.twist.linear.y = atof(parts[3].c_str());
    msg.twist.linear.z = ::atof(parts[4].c_str());
    // FOM equivalent to Standard deviation
    msg.variance = pow(::atof(parts[5].c_str()), 2);
    // DVL body translations
    // We still need to transform these translations
    // from DVL to world frame, before using it for localization.
    msg.translation.x = 0.5 * msg.twist.linear.x * msg.dt;
    msg.translation.y = 0.5 * msg.twist.linear.y * msg.dt;
    msg.translation.z = 0.5 * msg.twist.linear.z * msg.dt;
    msg.altitude = ::atof(parts[6].c_str());
    // Percent of confidence  (FOM max ~= 0.4)
    if (parts[7] == "y") {
      msg.confidence =
        100 * (1 - std::min(0.4, ::atof(parts[4].c_str())) / 0.4);
    } else {
      msg.confidence = 0;
    }
    // Temprature warring status
    if (parts[8] == "1") {
      msg.status = true;
    } else {
      msg.status = false;
    }
    count_++;
    msg.count = count_;

    _publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Done publishing:---- %i msgs ----",
      count_);

  } else {
    RCLCPP_INFO(this->get_logger(), " Not complete read :(");
  }
}

void VDVL::SerialNode::OnRead(
  std::vector<unsigned char> & buffer,
  size_t bytesRead)
{
  /* **********************************************************
   * OnRead callback is invoked to handle the received string buffer
   * before parsing.
   * ********************************************************** */
  std::string tmpBuff(buffer.begin(), buffer.begin() + bytesRead);

  _serial->get_serial_port().get_io_service().post(
    boost::bind(&SerialNode::OnPublish, this, tmpBuff));
}
