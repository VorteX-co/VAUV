// Copyright 2022 VorteX-co
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

#include "sinusoidal_trajectory.hpp"

SinusoidalTrajectory::SinusoidalTrajectory() {}
void SinusoidalTrajectory::set_params(
  const Vector3d & translation_constraints,
  const Vector3d & rotation_constraints)
{
  // setting veclocity, acceleration and jerk constraints
  translation_constraints_ = translation_constraints;
  rotation_constraints_ = rotation_constraints;
}
// =========================================================================================
void SinusoidalTrajectory::init(const double & t, const VectorXd & state)
{
  // Setting the start time of a sinusoidal trajectory
  trajectory_start_time_ = t;
  start_state_ = state;
}
// =========================================================================================
void SinusoidalTrajectory::generate_trajectory(
  const VectorXd & state, const VectorXd & reference,
  const double & t, VectorXd & pos,
  VectorXd & vel, VectorXd & acc)
{
  // Generating a sinusoidal states relative to the start time
  // Fixing z-axis and linear motion in  the x-axis
  double clock = t - trajectory_start_time_;
  // X axis
  pos(0) = start_state_(0) + 0.5 * clock;
  vel(0) = 0.5;
  acc(0) = 0.0;
  // Y axis
  pos(1) = start_state_(1) + std::sin(0.5 * clock);
  vel(1) = 0.5 * std::cos(0.5 * clock);
  acc(1) = -0.5 * 0.5 * std::sin(0.5);
  // Z axis
  pos(2) = start_state_(2) + 0.0 * clock;
  vel(2) = 0.0;
  acc(2) = 0.0;
  // orientation
  pos(3) = std::atan2(vel(1), vel(0));
  pos(4) = 0;
  pos(5) = 0;
  //  angular rate
  vel(3) = (vel(0) * acc(1) - vel(1) * acc(0)) / (vel(0) * vel(0) + vel(1) * vel(1));
  vel(4)  = 0;
  vel(5) = 0;
  // Minimizing the side-slip angle β
  vel(0) = std::sqrt(vel(0) * vel(0) + vel(1) * vel(1));
  vel(1) = 0;
}
// =========================================================================================
