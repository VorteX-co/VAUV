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
#ifndef SIMULTANEOUS_TRAJECTORY_HPP_
#define SIMULTANEOUS_TRAJECTORY_HPP_
#include <math.h>
#include <iostream>
#include "abstract_trajectory.hpp"
#include "auv_core_headers.hpp"
#include "min_jerk_trajectory.hpp"
#include "waypoint.hpp"


namespace auv_guidance
{
// Creates a trajectory to perform both translational and rotational motion
// simultaneously in a specified duration
class SimultaneousTrajectory : public Trajectory
{
private:
  MinJerkTrajectory * mjtX_, * mjtY_, * mjtZ_, * mjtAtt_;
  Waypoint * wStart_, * wEnd_;
  Eigen::Quaterniond qStart_, qEnd_, qRel_, qSlerp_;
  double totalDuration_, angularDistance_;

  Eigen::Vector3d xState_, yState_, zState_, angleState_;
  Eigen::Vector3d rotationAxis_;  // Axis for rotation wrt B-frame
  bool noRotation_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimultaneousTrajectory(Waypoint * start, Waypoint * end, double duration);
  void initTrajectory();
  double getDuration();
  auv_core::Vector13d computeState(double time);
  auv_core::Vector6d computeAccel(double time);
};
}  // namespace auv_guidance

#endif  // SIMULTANEOUS_TRAJECTORY_HPP_
