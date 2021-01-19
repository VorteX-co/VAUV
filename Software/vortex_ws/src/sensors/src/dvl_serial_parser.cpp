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

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "../include/dvl_companion/parser.hpp"

Parser::Parser(const std::string & packet)
: buffer(
    packet)               // init with input string reading, Parser p("wrx,131,..")
{
  setData();         // extracting measurements and setting the output variables
}

void Parser::setData()
{
  char firstLetter = buffer.at(0);
  if (firstLetter == 'w') {
    if (buffer.length() < 3) {
      std::cout << "sentance is too short " << std::endl;
    } else {
      int arr[8];
      int i;
      size_t found = 0;
      for (i = 0; i < 8; i++) {
        found = buffer.find(",", found + 1);
        arr[i] = found;
      }
      std::string time = buffer.substr(arr[0] + 1, arr[1] - arr[0] - 1);
      numerics.push_back(std::atof(time.c_str()));                   // from string to float
      std::string VX = buffer.substr(arr[1] + 1, arr[2] - arr[1] - 1);
      numerics.push_back(std::atof(VX.c_str()));
      std::string VY = buffer.substr(arr[2] + 1, arr[3] - arr[2] - 1);
      numerics.push_back(std::atof(VY.c_str()));
      std::string VZ = buffer.substr(arr[3] + 1, arr[4] - arr[3] - 1);
      numerics.push_back(std::atof(VZ.c_str()));
      std::string FOM = buffer.substr(arr[4] + 1, arr[5] - arr[4] - 1);
      numerics.push_back(std::atof(FOM.c_str()));
      std::string ALTITUDE = buffer.substr(arr[5] + 1, arr[6] - arr[5] - 1);
      numerics.push_back(std::atof(ALTITUDE.c_str()));
      std::string v = buffer.substr(arr[6] + 1, arr[7] - arr[6] - 1);
      valid = v[0];
      std::string s = buffer.substr(arr[7] + 1, 1);
      char sc = s[0];
      if (sc == '1') {
        status = true;
      } else {
        status = false;
      }
    }
  } else {
    std::cout << "invalid format " << std::endl;
  }
}

std::vector<float> Parser::getNumerics()    // retrieving numerical data
{
  return numerics;
}
bool Parser::getStatus()
{
  return status;
}
unsigned char Parser::getValid()
{
  return valid;
}
