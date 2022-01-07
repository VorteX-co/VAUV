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

#include "guidance.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

void Guidance::set_params(
  const Vector3d & translation_constraints,
  const Vector3d & rotation_constraints,
  const double & radius_of_acceptance)
{
  translation_constraints_ = translation_constraints;
  rotation_constraints_ = rotation_constraints;
  radius_of_acceptance_ = radius_of_acceptance;
  prev_goal_ << 0.0, 0.0;
  goal_ = prev_goal_;
  dt_ = 0.1;
  delta_ = 2.5;
  kappa_ = 0.2;
}
// =========================================================================================
void Guidance::los_setpoint(const Vector2d & p)
{
  prev_goal_ = goal_;
  goal_ = p;
}
// =========================================================================================
void Guidance::los_steering(const Vector2d & p, double & psi_des, double & r_des)
{
  double x = p(0);
  double y = p(1);
  double xk_prev = prev_goal_(0);
  double yk_prev = prev_goal_(1);
  double xk = goal_(0);
  double yk = goal_(1);
  // Path-tangential angle, eq. (10.55) p. 258 in Fossen2011.
  double alpha = atan2(yk - yk_prev, xk - xk_prev);
  // Alonge-track and cross-track errors, eq. (10.58, 10.59) p. 258 in
  // Fossen2011.
  double s = (x - xk) * cos(alpha) + (y - yk) * sin(alpha);
  double e = -(x - xk) * sin(alpha) + (y - yk) * cos(alpha);
  // ILOS guidance law
  double Kp = 1 / delta_;
  double Ki = kappa_ * Kp;
  // Velocity-path relative angle
  double chi_r = -atan(Kp * e + Ki * e_int_);
  // Desired Heading angle, eq. (10.72) p. 261 in Fossen2011.
  double psi_ref = alpha + chi_r;
  // Low-pass filter for smooth signal
  // T time contant vector for  yaw-angle
  double T = 5 * (1 / 0.1);
  psi_des_smooth_ += dt_ * (psi_ref - psi_des_smooth_) / T;
  psi_des = psi_des_smooth_;
  r_des = (psi_ref - psi_des_smooth_) / T;

  // cross-track error differential equation
  double e_int_dot =
    delta_ * e /
    (delta_ * delta_ + (e + kappa_ * e_int_) * (e + kappa_ * e_int_));
  e_int_ += e_int_dot * dt_;
}
// =========================================================================================
void Guidance::generate_trajectory(
  const Vector6d & state,
  const Vector6d & reference)
{
  /*
   * Reference: Jerk-limited Real-time Trajectory Generation with Arbitrary
   * Target States. github.com/pantor/ruckig
   */
  ruckig::InputParameter<6> input;
  // Motion state
  input.current_position = {state(0), state(1), state(2),
    state(3), state(4), state(5)};
  input.current_velocity = {
    /*
     * TODO [anyone] : current velocities is set to zeros assuming that the
     * veichle will stop at each point to do some operation, for generating a
     * trajectory from dynamic state to another set the current and the
     * reference velocities as well as accelerations
     */
    0, 0, 0, 0, 0, 0};
  input.current_acceleration = {0, 0, 0, 0, 0, 0};
  // Motion reference
  input.target_position = {reference(0), reference(1), reference(2),
    reference(3), reference(4), reference(5)};
  input.target_velocity = {0, 0, 0, 0, 0, 0};
  input.target_acceleration = {0, 0, 0, 0, 0, 0};
  // Motion constraints
  input.max_velocity = {
    translation_constraints_(0), translation_constraints_(0),
    translation_constraints_(0), rotation_constraints_(0),
    rotation_constraints_(0), rotation_constraints_(0)};
  input.max_acceleration = {
    translation_constraints_(1), translation_constraints_(1),
    translation_constraints_(1), rotation_constraints_(1),
    rotation_constraints_(1), rotation_constraints_(1)};
  input.max_jerk = {translation_constraints_(2), translation_constraints_(2),
    translation_constraints_(2), rotation_constraints_(2),
    rotation_constraints_(2), rotation_constraints_(2)};
  input.min_velocity = {
    -translation_constraints_(0), -translation_constraints_(0),
    -translation_constraints_(0), -rotation_constraints_(0),
    -rotation_constraints_(0), -rotation_constraints_(0)};
  input.min_acceleration = {
    -translation_constraints_(1), -translation_constraints_(1),
    -translation_constraints_(1), -rotation_constraints_(1),
    -rotation_constraints_(1), -rotation_constraints_(1)};
  // Synchronization is disabled so that each DoF stops as fast as possible
  // independently
  input.synchronization = ruckig::Synchronization::None;
  // Ruckig online trajectory generation
  ruckig::Ruckig<6> otg;
  // Compute time-optimal trajectory
  ruckig::Result result = otg.calculate(input, desired_trajectory_);
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Invalid input!" << std::endl;
  }
}
// =========================================================================================
std::tuple<Vector6d, Vector6d, Vector6d> Guidance::evaluate_desired_trajectory(
  const double & t)
{
  // Evaluating the desired trajectory at time t
  std::array<double, 6> desired_position, desired_velocity,
    desired_acceleration;
  desired_trajectory_.at_time(t, desired_position, desired_velocity,
    desired_acceleration);
  // Converting std::array to Eigen::Vector
  Vector6d smooth_acceleration;
  smooth_acceleration << desired_acceleration[0], desired_acceleration[1],
    desired_acceleration[2], desired_acceleration[3], desired_acceleration[4],
    desired_acceleration[5];
  Vector6d smooth_velocity;
  smooth_velocity << desired_velocity[0], desired_velocity[1],
    desired_velocity[2], desired_velocity[3], desired_velocity[4],
    desired_velocity[5];
  Vector6d smooth_position;
  smooth_position << desired_position[0], desired_position[1],
    desired_position[2], desired_position[3], desired_position[4],
    desired_position[5];
  // Desired motion state
  return std::make_tuple(smooth_position, smooth_velocity, smooth_acceleration);
}
