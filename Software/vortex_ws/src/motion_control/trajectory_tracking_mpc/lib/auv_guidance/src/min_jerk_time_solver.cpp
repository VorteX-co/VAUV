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
#include "min_jerk_time_solver.hpp"

namespace auv_guidance
{
/**
 * @param start Initial conditions of position, velocity, acceleration, and jerk
 * @param end Final conditions of position, velocity, acceleration, and jerk
 */
MinJerkTimeSolver::MinJerkTimeSolver(
  const Eigen::Ref<const Eigen::Vector4d> & start,
  const Eigen::Ref<const Eigen::Vector4d> & end)
{
  minTime_ = 0;
  problemMTTS_.AddResidualBlock(
    new ceres::AutoDiffCostFunction<MonotonicTrajectoryTimeSolver, 1, 1>(
      new MonotonicTrajectoryTimeSolver(start, end)),
    NULL, &minTime_);
  problemMTTS_.SetParameterLowerBound(&minTime_, 0, 0.0);
  optionsMTTS_.max_num_iterations = 100;
  optionsMTTS_.linear_solver_type = ceres::DENSE_QR;

  ceres::Solve(optionsMTTS_, &problemMTTS_, &summaryMTTS_);
}

/**
 * Returns the time calculated by the solver
 */
double MinJerkTimeSolver::getDuration() {return minTime_;}
}  // namespace auv_guidance
