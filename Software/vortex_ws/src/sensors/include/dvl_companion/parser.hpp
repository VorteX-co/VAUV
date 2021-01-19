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

#ifndef DVL_COMPANION__PARSER_HPP_
#define DVL_COMPANION__PARSER_HPP_
#pragma once
#include <string>
#include <vector>

class Parser
{
/* *****************************************************************
 * This class parses the received serial string by doig the following:
 * Stores the serial string as a private variable to be shared with the member
 * functions Stores the outupt dvl report as private variables to be assigned
 * by the member functions set the output variables from a function called
 * setData()
 ******************************************************************* */

private:
  std::string buffer;  // "wrx,112.83,0.007,0.017,0.006,0.000,0.93,y,0*d2"
  std::vector<float> numerics;  // any float numbers in the buffer
  void setData();
  bool status;          // 0 , 1 warrning for excesive temperature
  unsigned char valid;  // 'y' in water , 'n' in air

public:
  explicit Parser(const std::string & packet);
  std::vector<float> getNumerics();
  bool getStatus();
  unsigned char getValid();
};
#endif  // DVL_COMPANION__PARSER_HPP_
