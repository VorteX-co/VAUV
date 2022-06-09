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

#include "optimal_trajectory.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

Trajectory::Trajectory() {}
void Trajectory::set_params(
  const Vector3d & translation_constraints,
  const Vector3d & rotation_constraints)
{
  translation_constraints_ = translation_constraints;
  rotation_constraints_ = rotation_constraints;
}
// =========================================================================================
void Trajectory::generate_trajectory(const Vector12d & state, const Vector12d & reference)
{
  /*
   * Reference: Jerk-limited Real-time Trajectory Generation with Arbitrary
   * Target States. and intermediate states.
   *  github.com/pantor/ruckig
   */
  const size_t DOFs {6};
  ruckig::InputParameter<6> input;
  for (size_t i = 0; i < DOFs; i++) {
    // Current  state
    input.current_position[i] = state(i);
    input.current_velocity[i] = state(6 + i);
    // Reference state
    input.target_position[i] = reference(i);
    input.target_velocity[i] = reference(6 + i);
    input.target_acceleration[i] = 0.0;
    // Motion constraints
    input.max_velocity[i] = translation_constraints_(0);
    input.max_acceleration[i] = translation_constraints_(1);
    input.max_jerk[i] = translation_constraints_(2);
  }
  input.min_velocity = {-0.5, -0.5, -0.5, -0.3, -0.3, -0.3};
  input.min_acceleration = {-0.45, -0.45, -0.45, -0.35, -0.35, -0.25};
  // Synchronization is disabled so that each DoF stops as fast as possible
  // independently
//     input.synchronization = ruckig::Synchronization::None;
  // Ruckig online trajectory generation
  ruckig::Ruckig<DOFs> otg;
  // Compute time-optimal trajectory
  ruckig::Result result = otg.calculate(input, desired_trajectory_);
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Invalid input!" << std::endl;
  }
}
// =========================================================================================
void Trajectory::evaluate_generated_trajectory(
  const double & t, Vector6d & pos,
  Vector6d & vel, Vector6d & acc)
{
  const size_t DOFs {6};
  // Evaluating the desired trajectory at time t
  std::array<double, DOFs> desired_position, desired_velocity,
    desired_acceleration;
  desired_trajectory_.at_time(
    t, desired_position, desired_velocity,
    desired_acceleration);
  for (size_t i = 0; i < DOFs; i++) {
    pos(i) = desired_position[i];
    vel(i) = desired_velocity[i];
    acc(i) = desired_acceleration[i];
  }
}
