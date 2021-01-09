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

#ifndef DVL_COMPANION__LINUX_SERIAL_HPP_
#define DVL_COMPANION__LINUX_SERIAL_HPP_
#pragma once

#include <string>
#include <vector>

#ifdef __linux
using Tstring = std::string;
// this replacement is used to reduce lines length '<80 as cpplint imposes'
using Tchar = char;
class SerialInfo
{
private:
// "/dev/ttyUSB1"
  Tstring port_name;
// "dvl-a50"
  Tstring device;

public:
  const std::string port() const;
  const std::string device_name() const;
  SerialInfo();
  SerialInfo(const SerialInfo &);
  explicit SerialInfo(const std::string & name);
  SerialInfo(const std::string & name, const std::string & device_name);
};
#endif
std::vector<SerialInfo> getSerialList();

class Serial
{
public:
  struct Config    // defining a dtype for port information
  {unsigned int baudRate;
    unsigned int byteSize;
    enum class Parity
    {
      NO,           // NO parity
      EVEN,           // EVEN Parity
      ODD
    } parity;
    enum class StopBits
    {
      ONE,           // 1 bit
      ONE5,           // 1.5 bit
      TWO           // 2 bits
    } stopBits;};

private:
// Port information
  SerialInfo info;
  bool opened;
  Config conf;
// windows->handle
// linux->file descriptor & old termios file
  void * handle;

public:
  Serial();
  Serial(const Serial &) = delete;
  ~Serial();
  bool open(const Tstring & port_name, unsigned int baudRate = 115200);
  bool open(const SerialInfo & serial_info, unsigned int baudRate = 115200);
  void close();
  const Config & getConfig() const;          // get port information
  void setConfig(const Config &);            // set
  const SerialInfo & getInfo() const;        // get device information
  bool isOpened() const;                     // is the device is open?
  int read(unsigned char * data, int size);  // Receive , size of bytes/buffer
  unsigned char read1byte();                 // Receive 1 byte
  std::vector<unsigned char> read();         // Receive all buffers
  void clear();                              // clear the buffer
  void clearWrite();                         // clear output buffer
  void clearRead();                          // Clear input buffer
  int write(unsigned char * data, int size);  // Sending data to serial port
  int write(const std::vector<unsigned char> & data);
};
#endif  // DVL_COMPANION__LINUX_SERIAL_HPP_
