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
#ifndef SEGMENT_PLANNER_HPP_
#define SEGMENT_PLANNER_HPP_
#include <math.h>
#include <sstream>
#include "eigen3/Eigen/Dense"

using  Eigen;

namespace auv_guidance
{
// This class performs motion planning along a single axis using the distance to
// be traveled, with which is constrains velocity to a trapezoidal profile

class SegmentPlanner
{
private:
  double distance_, cruiseSpeed_, acceleration_;
  double cruiseDuration_, initialSpeed_, maxSpeed_, finalSpeed_;
  int accelSeq_;
  double t1_, t2_, tMid_,
    tEnd_;    // Key times. At cruiseSpeed for in the time interval [t1_, t2_]
  bool accelerate_;

public:
  static const int SEQ_NONE = 0;
  static const int SEQ_START = 1;
  static const int SEQ_END = 2;
  static const int SEQ_BOTH = 3;
  static const double DEFAULT_SPEED = 1.0;

  SegmentPlanner(
    double distance, double nominalSpeed, double accel = 0.0,
    int seq = SegmentPlanner::SEQ_NONE);
  void initMotionPlanner();
  double getTravelTime();
  Vector2d computeState(double t);
};
}  // namespace auv_guidance

#endif  // SEGMENT_PLANNER_HPP_
