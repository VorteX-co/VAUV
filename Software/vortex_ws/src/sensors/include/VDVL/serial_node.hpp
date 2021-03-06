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

#ifndef VDVL__SERIAL_NODE_HPP__
#define VDVL__SERIAL_NODE_HPP__
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <string>
#include <vector>
#include "custom_ros_interfaces/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.hpp"
namespace VDVL
{
class SerialNode : private boost::noncopyable,
  public boost::enable_shared_from_this<SerialNode>,
  public rclcpp::Node
{
  boost::shared_ptr<VDVL::SerialPort> _serial;
  std::string _portName;
  unsigned int _baudRate;
  boost::posix_time::ptime _lastRead;
  rclcpp::Publisher<custom_ros_interfaces::msg::DVL>::SharedPtr _publisher;
  unsigned int count_{0};
  void OnPublish(std::string buffer);
  rclcpp::Time _pub_time;

public:
  SerialNode();
  void OnRead(std::vector<unsigned char> & buffer, size_t bytesRead);
  void Create(boost::asio::io_service & ios);
};

}  // namespace VDVL
#endif  // VDVL__SERIAL_NODE_HPP__
