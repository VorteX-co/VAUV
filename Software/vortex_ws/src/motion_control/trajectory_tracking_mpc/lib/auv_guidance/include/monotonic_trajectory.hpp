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
#ifndef MONOTONIC_TRAJECTORY_HPP_
#define MONOTONIC_TRAJECTORY_HPP_
#include <math.h>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using Eigen;

namespace auv_guidance
{
// The monotonic trajectory time solver can fail if the signs on the velocity do
// not follow a general convention. This class takes care of that problem.
class MonotonicTrajectory
{
private:
  std::vector<MonotonicTrajectoryBase *> mtbList_;
  std::vector<double> mtbTimes_;

  Vector4d start_, end_;
  double totalTime_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MonotonicTrajectory(
    const Ref<const Vector4d> & start,
    const Ref<const Vector4d> & end, double maxAccel);
  double getTime();
  Vector3d computeState(double time);
  double getMiddleVelocity();
};
}  // namespace auv_guidance

#endif  // MONOTONIC_TRAJECTORY_HPP_
