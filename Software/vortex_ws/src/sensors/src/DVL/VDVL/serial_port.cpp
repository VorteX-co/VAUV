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
#include "../../../include/VDVL/serial_port.hpp"
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <iostream>
#include <vector>
boost::system::error_code VDVL::SerialPort::Flush()
{
  /* ***************************************************
   *  Clear all characters pending on the serial port
   *  especially when communication is first begun or upon an error condition.
   * *************************************************** */
  boost::system::error_code ec;
  const bool isFlushed = !::tcflush(_serialPort.native(), TCIOFLUSH);
  if (!isFlushed) {
    ec = boost::system::error_code(errno,
        boost::asio::error::get_system_category());
  }
  return ec;
}
void VDVL::SerialPort::SetErrorCode(const boost::system::error_code & ec)
{
  /* *****************************************************************
   * The method SetErrorCode() uses a mutex to protect the assignment of the
   * boost::system::error_code, as it is a composite object, and SetErrorCode()
   * can be called from multiple threads.
   * ***************************************************************** */
  if (ec) {
    boost::mutex::scoped_lock lock(_errorCodeMutex);
    _errorCode = ec;
  }
}
void VDVL::SerialPort::ReadBegin()
{
  /* ****************************************************************
   * ReadBegin method calls the async_read_until method with
   * a buffer to fill with incoming bytes from the port
   * a completetion handler callback is supplied to be invoked after
   * the reading process is finished
   * *************************************************************** */
  boost::asio::async_read_until(
    _serialPort, _readBuffer, '\n',
    boost::bind(&SerialPort::ReadComplete, shared_from_this(),
    boost::asio::placeholders::error,
    boost::asio::placeholders::bytes_transferred));
}
void VDVL::SerialPort::ReadComplete(
  const boost::system::error_code & ec,
  size_t bytesTransferred)
{
  /* ***************************************************************
   * ReadComplete callback is a data reading compeletion handler.
   * *************************************************************** */
  if (!ec) {  // No system error
    // copying the buffer to a local vector<unsigned char> for processing
    std::vector<unsigned char> tmpBuff(_readBuffer.size());
    buffer_copy(boost::asio::buffer(tmpBuff), _readBuffer.data());
    // invoking the OnRead callback to process the received data
    // callback executes before any additional reads.
    if (_onRead && (bytesTransferred > 0)) {
      _onRead(tmpBuff, bytesTransferred);
    }
    // clearing the buffer for another read.
    _readBuffer.consume(bytesTransferred);
    // queue another read.
    ReadBegin();
  } else {
    // boost::lexical_cast converting boost erros to string for displaying
    std::cout << boost::lexical_cast<std::string>(ec) << std::endl;
    // CLoseing the serial_port I/O object
    Close();
    SetErrorCode(ec);
  }
}
VDVL::SerialPort::SerialPort(
  boost::asio::io_service & ioService,
  const std::string & portName)
: _serialPort(ioService, portName), _isOpen(false) {}
void VDVL::SerialPort::Open(
  const boost::function<void(std::vector<unsigned char> &, size_t)> & onRead,
  unsigned int baudRate, VDVL::SerialParams serialParams,
  boost::asio::serial_port_base::flow_control flowControl)
{
  /* ***************************************************************
   * Open method sets the serial port cofiguration
   * also sets the user supplied Onread callback function
   * ***************************************************************/
  _onRead = onRead;
  _serialPort.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
  _serialPort.set_option(serialParams.get<0>());
  _serialPort.set_option(serialParams.get<1>());
  _serialPort.set_option(serialParams.get<2>());
  _serialPort.set_option(flowControl);
  /*
  const boost::system::error_code ec = Flush();
  if (ec) {
    SetErrorCode(ec);
  } */
  _isOpen = true;
  if (_onRead) {
    // don't start the async reader unless a read callback has been provided
    // be sure shared_from_this() is only called after object is already managed
    // in a shared_ptr, so need to fully construct, then call Open() on the ptr
    _serialPort.get_io_service().post(boost::bind(
        &SerialPort::ReadBegin,
        shared_from_this()));  // want read to start from a thread in io_service
  }
}
VDVL::SerialPort::~SerialPort() {Close();}
void VDVL::SerialPort::Close()
{
  if (_isOpen) {
    _isOpen = false;
    boost::system::error_code ec;
    _serialPort.cancel(ec);
    ec = Flush();
    SetErrorCode(ec);
    _serialPort.close(ec);
    SetErrorCode(ec);
  }
}
