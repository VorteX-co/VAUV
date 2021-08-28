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
#ifndef MIN_JERK_TRAJECTORY_HPP_
#define MIN_JERK_TRAJECTORY_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace auv_guidance
{
class MinJerkTrajectory
{
private:
  double c0_, c1_, c2_, c3_, c4_, c5_;  // Polynomial coefficients
  double dt, dt2;
  double x0_, v0_, a0_, xf_, vf_, af_;  // Initial and final conditions
  double t0_, tf_;

public:
  MinJerkTrajectory(
    const Eigen::Ref<const Eigen::Vector3d> & start,
    const Eigen::Ref<const Eigen::Vector3d> & end,
    double duration);
  void computeCoeffs();
  Eigen::Vector3d computeState(double time);
  double getMiddleVelocity();
};
}  // namespace auv_guidance

#endif  // MIN_JERK_TRAJECTORY_HPP_
