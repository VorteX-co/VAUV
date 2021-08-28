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
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>
#include <vector>
#include "../../include/VDVL/executor.hpp"
#include "../../include/VDVL/serial_node.hpp"
#include "rclcpp/rclcpp.hpp"

// cout_lock mutex prevents simulatenous calling to Log callback
boost::mutex cout_lock;
void Log(const std::string & msg)
{
  /* ***************************************
   * Log Callback function used to display error messages from threads
   * *************************************** */
  boost::mutex::scoped_lock lock(cout_lock);
  std::cout << "[" << boost::this_thread::get_id() << "] " << msg << std::endl;
}
/* --------------------------------------------*/
int main(int argc, char ** argv)
{
  std::string port_name;
  int64_t baud_rate;
  rclcpp::init(argc, argv);
  const boost::shared_ptr<VDVL::SerialNode> node(new VDVL::SerialNode());
  node->get_parameter("portName", port_name);
  node->get_parameter("baudRare", baud_rate);
  // Initializing an Executor for launching thread pool of worker threads.
  VDVL::Executor exe;
  // Setting the OnError callback for error handling during execution
  exe.OnError = [](boost::asio::io_service &, boost::system::error_code ec) {
      /* ********************************
       * Error handling callback as lambda expression
       *
       * ******************************** */
      Log(std::string("SerialNode error (asio): ") +
        boost::lexical_cast<std::string>(ec));
      rclcpp::shutdown();
      exit(-1);
    };
  // Setting the OnExeption callback for exception handling during execution
  exe.OnException = [](boost::asio::io_service &, const std::exception & ex) {
      /* ********************************
       * Exception handling callback as lambda expression
       *
       * ******************************** */
      Log(std::string("SerialNode exception (asio): ") + ex.what());
    };
  // Setting the OnRun callback of the executor to Create method of the
  // SerialNode Class to Start the serial port connection.
  exe.OnRun =
    boost::bind(&VDVL::SerialNode::Create, node, boost::ref(exe.GetIO()));
  // Joining <N> worker threads.
  exe.Run(2);
  exit(0);
}
