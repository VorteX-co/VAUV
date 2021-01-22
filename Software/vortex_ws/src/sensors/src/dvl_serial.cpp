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

#include <memory>
#include <string>
#include <vector>
#include "../include/dvl_companion/linux_serial.hpp"
#include "../include/dvl_companion/parser.hpp"
#include "custom_ros_interfaces/msg/dvl.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

Serial serial;  // Serial port instance

class Publisher : public rclcpp::Node
{
public:
  inline Publisher();
  inline void publish(std::string packet);

private:
  rclcpp::Publisher<custom_ros_interfaces::msg::DVL>::SharedPtr publisher_;
};
Publisher::Publisher()
: Node("a50_serial_pub")
{
  this->declare_parameter<std::string>("Port", "/dev/ttyUSB1");
  // declares a rclcpp param called Port with a default value
  this->declare_parameter<std::int64_t>("Baud_Rate", 115200);
  std::string topic = "dvl_report";
  publisher_ =
    this->create_publisher<custom_ros_interfaces::msg::DVL>(topic, 10);
}
void Publisher::publish(std::string packet)
{
  auto report = custom_ros_interfaces::msg::DVL();
  Parser parser(packet);
  std::vector<float> numerics = parser.getNumerics();
  report.time = numerics[0];
  report.vx = numerics[1];
  report.vy = numerics[2];
  report.vz = numerics[3];
  report.fom = numerics[4];
  report.altitude = numerics[5];
  report.status = parser.getStatus();
  report.valid = parser.getValid();
  publisher_->publish(report);
}
int main(int argc, char ** argv)
{
  // /dev/tty****
  std::string port_name;
  unsigned int baud_rate;
  // the return type of read() is vector
  std::vector<unsigned char> buffer;
  // init rclcpp params using terminal args.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Publisher>();
  // if Port param passed as arg. store it in port_name
  // else get the default Port value /dev/ttyUSB1
  node->get_parameter("Port", port_name);
  node->get_parameter("Baud_Rate", baud_rate);
  if (!serial.open(port_name, baud_rate)) {
    RCLCPP_ERROR(node->get_logger(), "couldn't open port,");
    rclcpp::shutdown();
    // return -1;
    exit(-1);
  }

  // rclcpp::spin(node);

  while (rclcpp::ok()) {       // whenever a buffer is available we read it
    buffer = serial.read();
    // from vector<unisgned char> to string
    std::string sentance(buffer.begin(), buffer.end());
    node->publish(sentance);
    RCLCPP_DEBUG(node->get_logger(), "published");
    rclcpp::spin_some(node);
  }
  // serial.close();
  // rclcpp::shutdown();
  exit(0);
}
