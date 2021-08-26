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

#ifndef VDVL__SERIAL_PORT_HPP__
#define VDVL__SERIAL_PORT_HPP__
#pragma once
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>
#include <string>
#include <vector>
namespace VDVL
{
/* ************************************************************
 *  Configuration attribute sets as Boost tuples,
 *  and create them with boost::make_tuple().
 * The default is 8N1, where 8 is the character size
 * N for no parity
 * 1 is the number of stop bits
 * ************************************************************/
typedef boost::tuple<boost::asio::serial_port_base::character_size,
    boost::asio::serial_port_base::parity,
    boost::asio::serial_port_base::stop_bits>
  SerialParams;

const SerialParams SP_8N1 =
  boost::make_tuple(8, boost::asio::serial_port_base::parity::none,
    boost::asio::serial_port_base::stop_bits::one);
/* *************************************************************
 * The SerialPort Class owns the I/O object [serial_port]
 * and manages the interfaces to the physical serial port
 * ************************************************************* */
class SerialPort : private boost::noncopyable,
  public boost::enable_shared_from_this<SerialPort>
{
  // serial_port I/O object holds services for interfacing the serial port
  boost::asio::serial_port _serialPort;
  bool _isOpen;
  boost::system::error_code _errorCode;
  boost::mutex _errorCodeMutex;
  boost::asio::streambuf _readBuffer;
  // _onRead variable used to hold external callback function supplied
  // by the user of the SerialPort class [when using an object from this
  // class we can supply
  boost::function<void(std::vector<unsigned char> &, size_t)> _onRead;
  // Flush method used to clear all characters pending on the serial port
  boost::system::error_code Flush();
  // SetError Code method
  // uses a mutex to protect the assignment of the boost::system::error_code
  void SetErrorCode(const boost::system::error_code & ec);
  // ReadBegin callback invoked to begin reading from port
  void ReadBegin();
  // ReadComplete callback invoked after each read process
  void ReadComplete(
    const boost::system::error_code & ec,
    size_t bytesTransferred);

public:
  SerialPort(boost::asio::io_service & ioService, const std::string & portName);
  ~SerialPort();

  void Open(
    const boost::function<void(std::vector<unsigned char> &, size_t)> & onRead,
    unsigned int baudRate = 115200, SerialParams serialParams = SP_8N1,
    boost::asio::serial_port_base::flow_control flowControl =
    boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
  void Close();
  boost::asio::serial_port & get_serial_port() {return _serialPort;}
};
}  // namespace VDVL

#endif  // VDVL__SERIAL_PORT_HPP__
