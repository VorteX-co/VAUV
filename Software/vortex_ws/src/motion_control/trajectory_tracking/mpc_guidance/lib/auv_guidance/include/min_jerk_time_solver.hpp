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
#ifndef MIN_JERK_TIME_SOLVER_HPP_
#define MIN_JERK_TIME_SOLVER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "monotonic_trajectory_time_solver.hpp"

namespace auv_guidance
{
// Solve for optimal time between two points given initial/final velocity,
// accel, and jerk
class MinJerkTimeSolver
{
private:
  // Ceres Problem
  ceres::Problem problemMTTS_;
  ceres::Solver::Options optionsMTTS_;
  ceres::Solver::Summary summaryMTTS_;
  double minTime_;

public:
  MinJerkTimeSolver(
    const Eigen::Ref<const Eigen::Vector4d> & start,
    const Eigen::Ref<const Eigen::Vector4d> & end);
  double getDuration();
};
}  // namespace auv_guidance

#endif  // MIN_JERK_TIME_SOLVER_HPP_
