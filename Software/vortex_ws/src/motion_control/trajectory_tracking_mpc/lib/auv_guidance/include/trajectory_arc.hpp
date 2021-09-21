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
#ifndef TRAJECTORY_ARC_HPP_
#define TRAJECTORY_ARC_HPP_
#include <math.h>
#include "abstract_trajectory.hpp"
#include "eigen3/Eigen/Dense"
#include "segment_planner.hpp"

using  Eigen;

namespace AUV_GNC
{
namespace Trajectory
{
typedef Matrix<float, 3, 2> Matrix32f;

// Creates an arc segment in space using the SegmentPlanner for position/speed
// along the segment. Assumes the speed is either zero or equal to the speed
// entered in the constructor
class Arc : public TrajectoryGenerator
{
private:
  SegmentPlanner * segPlanner_;
  float cruiseSpeed_, acceleration_;
  int accelSeq_;
  Vector3f initialPos_, unitTangent_, unitNormal_, unitAxis_;
  float radius_, theta_;
  Matrix32f insertionMap_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const float DEFAULT_SPEED = 0.5;  // [m/s]
  static const float DEFAULT_ACCEL = 0.2;  // [m/s^2]

  Arc(
    const Ref<const Vector3f> initialPos,
    const Ref<const Vector3f> unitTangent,
    const Ref<const Vector3f> unitNormal, float radius, float theta,
    float nominalSpeed = Arc::DEFAULT_SPEED, float acceleration = 0.0,
    int seq = SegmentPlanner::SEQ_NONE);
  float getTravelTime();
  Vector12f computeState(float time);
};
}  // namespace Trajectory
}  // namespace AUV_GNC

#endif  // TRAJECTORY_ARC_HPP_
