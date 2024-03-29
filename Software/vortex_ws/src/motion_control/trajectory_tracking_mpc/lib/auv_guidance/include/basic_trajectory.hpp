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
#ifndef BASIC_TRAJECTORY_HPP_
#define BASIC_TRAJECTORY_HPP_
#include <math.h>
#include <algorithm>

#include "abstract_trajectory.hpp"
#include "auv_core_headers.hpp"
#include "long_trajectory.hpp"
#include "min_jerk_time_solver.hpp"
#include "min_jerk_trajectory.hpp"
#include "simultaneous_trajectory.hpp"
#include "tgen_limits.hpp"
#include "waypoint.hpp"

namespace auv_guidance
{
class BasicTrajectory : public Trajectory
{
private:
  auv_core::auvConstraints * auvConstraints_;
  SimultaneousTrajectory * stStop_, * stPrimary_;
  LongTrajectory * ltPrimary_;
  MinJerkTrajectory * mjt_;
  Waypoint * wStart_, * wStop_, * wEnd_;
  Eigen::Quaterniond qStop_, qEnd_;

  Eigen::Vector3d unitVec_, deltaVec_, maxVelocityVec_;
  double totalDuration_, stopDuration_, simultaneousDuration_, longDuration_;
  double distance_, maxVelocityST_, maxVelocity_;

  bool isLongTrajectory_, isSimultaneousTrajectory_, exceedsMaxSpeed_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicTrajectory(
    auv_core::auvConstraints * constraints, Waypoint * wStart,
    Waypoint * wEnd);
  void setStopTrajectory();
  void computeMaxVelocityST();
  void computeSimultaneousDuration();
  void setPrimaryTrajectory();
  double getDuration();
  auv_core::Vector13d computeState(double time);
  auv_core::Vector6d computeAccel(double time);
};
}  // namespace auv_guidance

#endif  // BASIC_TRAJECTORY_HPP_
