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

#include "steering.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

void Steering::set_params(
  const double & radius_of_acceptance, const double & dt,
  const double & kappa, const double & delta)
{
  // Radius of a circle around the goal waypoints
  radius_of_acceptance_ = radius_of_acceptance;
  // Previous goal waypoint (x_i, y_i)
  prev_goal_ << 0.0, 0.0;
  // Goal waypoint (x_i+1 , y_i+1)
  goal_ = prev_goal_;
  // Sampling time
  dt_ = dt;
  // Δ Lookahead distance
  // Kp = 1 / Δ
  delta_ = delta;
  // κ Integral action gain
  // Ki =  κ Kp
  kappa_ = kappa;
}
// =========================================================================================
void Steering::setpoint(const Vector2d & p)
{
  prev_goal_ = goal_;
  goal_ = p;
}
// =========================================================================================
void Steering::compute_heading(
  const Vector2d & position, double & psi_des, double & r_des)
{
  double x = position(0);
  double y = position(1);
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
  // Kp = 1 / Δ
  double Kp = 1 / delta_;
  // Ki =  κ Kp
  double Ki = kappa_ * Kp;
  // Velocity-path relative angle
  // with added Integral action Ki * e_int_
  double chi_r = -atan(Kp * e + Ki * e_int_);
  // Desired Heading angle, eq. (10.72) p. 261 in Fossen2011.
  double psi_ref = alpha + chi_r;
  // Low-pass filter for smooth signal
  // T time contant vector for  yaw-angle
  double T = 5 * (1 / 0.1);
  psi_des_smooth_ += dt_ * (psi_ref - psi_des_smooth_) / T;
  psi_des = psi_des_smooth_;
  r_des = (psi_ref - psi_des_smooth_) / T;

  // Cross-track error differential equation
  // Nonlinear ILOS (Børhaug et al. 2008)
  double e_int_dot = delta_ * e /
    (delta_ * delta_ + (e + kappa_ * e_int_) * (e + kappa_ * e_int_));
  e_int_ += e_int_dot * dt_;
}
