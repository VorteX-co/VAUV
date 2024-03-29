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
#ifndef ABSTRACT_TRAJECTORY_HPP_
#define ABSTRACT_TRAJECTORY_HPP_

#include "eigen_typedefs.hpp"

namespace auv_guidance
{
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * \brief This is a pure virtual class. All methods MUST be declared in
 * inheriting classes/
 */
class Trajectory
{
public:
  virtual auv_core::Vector13d computeState(double time) = 0;
  virtual auv_core::Vector6d computeAccel(double time) = 0;
};
}  // namespace auv_guidance

#endif  // ABSTRACT_TRAJECTORY_HPP_
