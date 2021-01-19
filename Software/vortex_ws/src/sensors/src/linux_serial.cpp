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

#ifdef __linux
#include "../include/dvl_companion/linux_serial.hpp"
#include <dirent.h>
#include <fcntl.h>
#include <libgen.h>
#include <linux/limits.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// /dev/tty***
const char no_device[]{"no device"};
const char unknown[]{"unknown_device"};
using PID = std::unordered_map<std::string, std::string>;
using database_map = std::unordered_map<Tstring, std::pair<Tstring, PID>>;
std::vector<std::string> split(std::string str, const std::string & sep)
{
  std::vector<std::string> v;
  while (str.size() > 0) {
    uint64_t place = str.rfind(sep);
    if (place != std::string::npos) {
      if (place + 1 == str.size()) {
        str.pop_back();
        continue;
      }
      v.push_back(&str.c_str()[place + sep.size()]);
      while (str.size() != place) {
        str.pop_back();
      }
    } else {
      v.push_back(str.c_str());
      break;
    }
  }
  std::reverse(v.begin(), v.end());
  return v;
}
class DataBase
{
private:
  database_map data;

public:
  DataBase()
  {
    std::ifstream ifs("/lib/udev/hwdb.d/20-usb-vendor-model.hwdb");
    if (!ifs) {return;}
    char buff[1024];
    std::vector<std::string> splited;
    Tstring pid;
    Tstring vid;
    Tstring vendor;
    while (!ifs.getline(buff, 1024).eof()) {
      switch (buff[0]) {
        case ' ':
          splited = split(&buff[1], "=");
          // product name
          if (splited[0] == "ID_MODEL_FROM_DATABASE") {
            data[vid].second[pid] = splited[1];
          } else if (splited[0] == "ID_VENDOR_FROM_DATABASE") {
            data[vid].first = splited[1];
          }
          break;
        case '\n':
        case '#':
        case '\0':
          continue;
          break;
        default:
          splited = split(buff, ":");
          if (splited[0] == "usb") {
            splited = split(splited[1], "p");
            // vendor
            if (splited.size() == 1) {
              splited[0].pop_back();
              vid = &splited[0].c_str()[1];
            } else {
              vid = &splited[0].c_str()[1];
              splited[1].pop_back();
              pid = splited[1].c_str();
            }
          }
          break;
      }
    }
    ifs.close();
  }
  std::string getName(const std::string & arg_vid, const std::string & arg_pid)
  {
    Tstring vid = arg_vid;
    transform(vid.begin(), vid.end(), vid.begin(), ::toupper);
    Tstring pid = arg_pid;
    transform(pid.begin(), pid.end(), pid.begin(), ::toupper);
    Tstring vender = data[vid].first;
    Tstring product = data[vid].second[pid];
    if (vid.size() > 0) {
      return vender + "  " + product;
    } else {
      return unknown;
    }
  }
} usb_database;
const std::string SerialInfo::port() const
{
  return port_name;
}
const std::string SerialInfo::device_name() const
{
  return device;
}
SerialInfo::SerialInfo()
{
  port_name = no_device;
  device = no_device;
}
SerialInfo::SerialInfo(const SerialInfo & info)
: port_name(info.port_name), device(info.device)
{
}
void find(const Tstring & p, const Tstring & sn, std::vector<Tstring> fn)
{
  struct dirent ** name_list = nullptr;
  int count = scandir(p.c_str(), &name_list, NULL, NULL);
  if (count == -1) {
    return;
  } else {
    struct stat stat_buff;
    Tstring search_path;
    for (int i = 0; i < count; i++) {
      bool c1 = Tstring("..").compare(name_list[i]->d_name);
      bool c2 = Tstring(".").compare(name_list[i]->d_name);
      if (c1 && c2) {
        search_path = p + std::string(name_list[i]->d_name);
        if (!sn.compare(name_list[i]->d_name)) {
          fn.push_back(search_path);
        }
        if (lstat(search_path.c_str(), &stat_buff) == 0) {
          auto is_dir = stat_buff.st_mode & S_IFMT;
          if (S_ISDIR(is_dir)) {
            find(search_path + "/", sn, fn);
          }
        }
      }
    }
  }
  free(name_list);
}
SerialInfo::SerialInfo(const std::string & _port)
: port_name(_port)
{
  char * _p = const_cast<char *>(_port.c_str());
  Tstring dev_name = basename(reinterpret_cast<char *>(_p));
  std::vector<std::string> paths;
  find("/sys/devices/", dev_name, paths);
  if (paths.size() == 0) {
    device = no_device;
    return;
  }
  Tstring path = paths[0];
  char buffer[1024];
  if (path.find("usb") != std::string::npos) {
    Tstring base;
    Tstring vid;
    Tstring pid;
    for (auto i = 0; i < 2; i++) {
      base = basename(reinterpret_cast<char *>(const_cast<char *>(path.c_str())));
      path.erase(path.end() - base.size() - 1, path.end());
    }
    std::ifstream ifs(path + "/idVendor");
    if (!ifs) {
      device = unknown;
      return;
    }
    ifs.getline(buffer, 1024);
    ifs.close();
    vid = buffer;
    ifs.open(path + "/idProduct");
    if (!ifs) {
      device = unknown;
      return;
    }
    ifs.getline(buffer, 1024);
    ifs.close();
    pid = buffer;
    device = usb_database.getName(vid, pid);
  } else {
    // port_name = no_device;
    device = unknown;
  }
}
SerialInfo::SerialInfo(const Tstring & _port, const Tstring & _device_name)
: port_name(_port), device(_device_name)
{
}
// ----------------------------------------------------------------------
// getSerialList
const std::string get_driver(const std::string & dir)
{
  struct stat st;
  Tstring devicedir = dir + "/device";
  if (lstat(devicedir.c_str(), &st) == 0 && S_ISLNK(st.st_mode)) {
    char buffer[1024];
    for (int i = 0; i < 1024; i++) {
      buffer[i] = 0;
    }
    devicedir += "/driver";
    if (readlink(devicedir.c_str(), buffer, sizeof(buffer)) > 0) {
      return basename(buffer);
    }
  }
  return "";
}
void probe_serial8250(std::vector<Tstring> pL, std::vector<Tstring> p8)
{
  struct serial_struct serialInfo;
  for (auto com8250 : p8) {
    int fd = open(com8250.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (fd >= 0) {
      if (ioctl(fd, TIOCGSERIAL, &serialInfo) == 0) {
        if (serialInfo.type != PORT_UNKNOWN) {pL.push_back(com8250);}
      }
      close(fd);
    }
  }
}
std::vector<SerialInfo> getSerialList()
{
  struct dirent ** nameList;
  std::vector<std::string> portList;
  std::vector<std::string> portList8250;
  const Tstring sysdir = "/sys/class/tty/";
  int listsize = scandir(sysdir.c_str(), &nameList, NULL, NULL);
  if (listsize < 0) {
    return std::vector<SerialInfo>();
  } else {
    for (int n = 0; n < listsize; n++) {
      bool c1 = Tstring("..").compare(nameList[n]->d_name);
      bool c2 = Tstring(".").compare(nameList[n]->d_name);
      if (c1 && c2) {
        Tstring devicedir = sysdir;
        devicedir += nameList[n]->d_name;
        Tstring driver = get_driver(devicedir);
        if (driver.size() > 0) {
          char * _d = const_cast<char *>(devicedir.c_str());
          Tstring s(basename(reinterpret_cast<char *>(_d)));
          Tstring devfile = "/dev/" + s;
          if (driver == "serial8250") {
            portList8250.push_back(devfile);
          } else {
            portList.push_back(devfile);
          }
        }
      }
      free(nameList[n]);
    }
    free(nameList);
  }
  probe_serial8250(portList, portList8250);
  std::vector<SerialInfo> list;
  for (auto port : portList) {
    list.push_back(SerialInfo(port));
  }
  return list;
}
// --------------------------------------------------------------------------
// Serial
const Serial::Config defconf = {
  9600,
  8,
  Serial::Config::Parity::NO,
  Serial::Config::StopBits::ONE,
};
Serial::Serial()
{
  conf = defconf;
  opened = false;
  handle = nullptr;
}
Serial::~Serial()
{
  close();
}
int lopen(const char * _file, int _oflag)
{
  return open(_file, _oflag);
}
int lclose(int fd)
{
  return close(fd);
}
int lread(int fd, void * buff, uint64_t size)
{
  return static_cast<int>(read(fd, buff, size));
}
int lwrite(int fd, void * buff, uint64_t size)
{
  return static_cast<int>(write(fd, buff, size));
}
struct descriptor_old_termios
{
  struct termios tms;
  int fd;
};
bool Serial::open(const std::string & port_name, unsigned int baudRate)
{
  return open(SerialInfo(port_name), baudRate);
}
bool Serial::open(const SerialInfo & serial_info, unsigned int baudRate)
{
  info = serial_info;
  conf = defconf;
  conf.baudRate = baudRate;
  handle = new descriptor_old_termios;
  int & fd = (reinterpret_cast<descriptor_old_termios *>(handle))->fd;
  struct termios tms;
  fd = lopen(info.port().c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    delete (reinterpret_cast<descriptor_old_termios *>(handle));
    return false;
  }
  // old
  tcgetattr(fd, &(reinterpret_cast<descriptor_old_termios *>(handle))->tms);
  tms = (reinterpret_cast<descriptor_old_termios *>(handle))->tms;
  tms.c_iflag = 0;
  tms.c_oflag = 0;
  tms.c_lflag = 0;
  tms.c_cc[VMIN] = 1;
  tms.c_cc[VTIME] = 0;
  cfmakeraw(&tms);
  cfsetispeed(&tms, baudRate);
  cfsetospeed(&tms, baudRate);
  switch (conf.parity) {
    case Config::Parity::NO:
      tms.c_cflag &= ~PARENB;
      tms.c_cflag &= ~PARODD;
      break;
    case Config::Parity::EVEN:
      tms.c_cflag |= PARENB;
      tms.c_cflag &= ~PARODD;
      break;
    case Config::Parity::ODD:
      tms.c_cflag |= PARENB;
      tms.c_cflag |= PARODD;
      break;
  }
  tms.c_cflag &= ~CSIZE;
  switch (conf.byteSize) {
    case 5:
      tms.c_cflag |= CS5;
      break;
    case 6:
      tms.c_cflag |= CS6;
      break;
    case 7:
      tms.c_cflag |= CS7;
      break;
    default:
      tms.c_cflag |= CS8;
      break;
  }
  switch (conf.stopBits) {
    case Config::StopBits::TWO:
      tms.c_cflag |= CSTOPB;
      break;
    default:
      tms.c_cflag &= ~CSTOPB;
      break;
  }
  // new
  // tcsetattr(fd, TCSANOW, &tms);
  tcsetattr(fd, TCSANOW, &tms);
  clear();
  opened = true;
  return true;
}
void Serial::close()
{
  descriptor_old_termios * h = reinterpret_cast<descriptor_old_termios *>(handle);
  descriptor_old_termios * pointer = h;
  tcsetattr(pointer->fd, TCSANOW, &(pointer->tms));
  lclose(pointer->fd);
  delete (reinterpret_cast<descriptor_old_termios *>(handle));
  opened = false;
}
const Serial::Config & Serial::getConfig() const
{
  return conf;
}
void Serial::setConfig(const Config & _conf)
{
  conf = _conf;
  if (!opened) {return;}
  int & fd = (reinterpret_cast<descriptor_old_termios *>(handle))->fd;
  struct termios tms;
  tcgetattr(fd, &tms);
  switch (conf.parity) {
    case Config::Parity::NO:
      tms.c_cflag &= ~PARENB;
      tms.c_cflag &= ~PARODD;
      break;
    case Config::Parity::EVEN:
      tms.c_cflag |= PARENB;
      tms.c_cflag &= ~PARODD;
      break;
    case Config::Parity::ODD:
      tms.c_cflag |= PARENB;
      tms.c_cflag |= PARODD;
      break;
  }
  tms.c_cflag &= ~CSIZE;
  switch (conf.byteSize) {
    case 5:
      tms.c_cflag |= CS5;
      break;
    case 6:
      tms.c_cflag |= CS6;
      break;
    case 7:
      tms.c_cflag |= CS7;
      break;
    default:
      tms.c_cflag |= CS8;
      break;
  }
  switch (conf.stopBits) {
    case Config::StopBits::TWO:
      tms.c_cflag |= CSTOPB;
      break;
    default:
      tms.c_cflag &= ~CSTOPB;
      break;
  }
  cfsetispeed(&tms, conf.baudRate);
  cfsetospeed(&tms, conf.baudRate);
  // new
  tcsetattr(fd, TCSANOW, &tms);
}
const SerialInfo & Serial::getInfo() const
{
  return info;
}
bool Serial::isOpened() const
{
  return opened;
}
int Serial::read(unsigned char * data, int size)
{
  descriptor_old_termios * h = reinterpret_cast<descriptor_old_termios *>(handle);
  int error = lread(h->fd, data, size);
  return error;
}
unsigned char Serial::read1byte()
{
  unsigned char data;
  lread((reinterpret_cast<descriptor_old_termios *>(handle))->fd, &data, 1);
  return data;
}
std::vector<unsigned char> Serial::read()
{
  std::vector<unsigned char> data;
  unsigned char * buffer = new unsigned char[1024];
  int size;
  size = read(buffer, 1024);
  if (size <= 0) {
    delete[] buffer;
    // data.push_back(read1byte());
    return data;
  }
  data.reserve(size);
  for (int i = 0; i < size; i++) {
    data.push_back(buffer[i]);
  }
  delete[] buffer;
  return data;
}
void Serial::clear()
{
  int & fd = (reinterpret_cast<descriptor_old_termios *>(handle))->fd;
  tcflush(fd, TCIOFLUSH);
}
void Serial::clearWrite()
{
  int & fd = (reinterpret_cast<descriptor_old_termios *>(handle))->fd;
  tcflush(fd, TCOFLUSH);
}
void Serial::clearRead()
{
  int & fd = (reinterpret_cast<descriptor_old_termios *>(handle))->fd;
  tcflush(fd, TCIFLUSH);
}
int Serial::write(unsigned char * data, int size)
{
  descriptor_old_termios * h = reinterpret_cast<descriptor_old_termios *>(handle);
  return lwrite(h->fd, data, size);
}
int Serial::write(const std::vector<unsigned char> & data)
{
  int size = static_cast<int>(data.size());
  unsigned char * buffer = new unsigned char[data.size()];
  for (int i = 0; i < size; i++) {
    buffer[i] = data[i];
  }
  size = write(buffer, size);
  delete[] buffer;
  return size;
}
#endif
