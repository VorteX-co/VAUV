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
#ifndef LONG_TRAJECTORY_HPP_
#define LONG_TRAJECTORY_HPP_
#include <math.h>
#include <vector>
#include "abstract_trajectory.hpp"
#include "auv_core_headers.hpp"
#include "min_jerk_time_solver.hpp"
#include "min_jerk_trajectory.hpp"
#include "simultaneous_trajectory.hpp"
#include "tgen_limits.hpp"
#include "waypoint.hpp"


namespace auv_guidance
{
// Defines a trajectory occuring over a long distance. Contains three travel
// phases: speed up, cruise, and slow down Speed up and slow down are both
// mirrored S-curves about the cruise region (assumes vehicle starts and ends at
// rest)
class LongTrajectory : public Trajectory
{
private:
  SimultaneousTrajectory * stPreRotation_, * stSpeedUp_, * stCruise_, * stSlowDown_,
    * stPostRotation_;
  auv_core::auvConstraints * auvConstraints_;
  std::vector<SimultaneousTrajectory *> stList_;
  std::vector<double> stTimes_;
  Waypoint * wStart_, * wEnd_, * wPreTranslate_, * wCruiseStart_, * wCruiseEnd_,
    * wPostTranslate_;
  Eigen::Quaterniond qStart_, qEnd_, qCruise_;

  Eigen::Vector3d unitVec_, deltaVec_, cruiseStartPos_, cruiseEndPos_,
    cruiseVel_;
  double totalDuration_, rotationDuration1_, rotationDuration2_, accelDuration_,
    cruiseDuration_;
  double cruiseRatio_, cruiseSpeed_;
  bool newTravelHeading_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LongTrajectory(
    Waypoint * start, Waypoint * end,
    auv_core::auvConstraints * constraints, double cruiseRatio,
    double cruiseSpeed);
  void initTrajectory();
  void initWaypoints();
  void initSimultaneousTrajectories();
  double computeRotationDuration(Eigen::Quaterniond qRel);
  double getDuration();
  auv_core::Vector13d computeState(double time);
  auv_core::Vector6d computeAccel(double time);
};
}  // namespace auv_guidance

#endif  // LONG_TRAJECTORY_HPP_
