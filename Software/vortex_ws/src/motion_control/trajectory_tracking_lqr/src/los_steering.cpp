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

#include "los_steering.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

void ILOS::set_params(
  const double & delta, const double & R, const double & dt,
  const double & kappa, const double & T)
{
  delta_ = delta;
  R_ = R;
  dt_ = dt;
  kappa_ = kappa;
  T_ = T;
}
void ILOS::setpoint(const Eigen::Vector2d & wpt) {wpts_.push_back(wpt);}
void ILOS::setpoints(const std::vector<Eigen::Vector2d> & wpts)
{
  wpts_.insert(wpts_.end(), wpts.begin(), wpts.end());
}
double ILOS::calculate_reference(const Eigen::Vector3d & planner_pose)
{
  double x = planner_pose(0);
  double y = planner_pose(1);
  double xk = wpts_[k_](0);
  double yk = wpts_[k_](1);
  double xk_next = wpts_[k_ + 1](0);
  double yk_next = wpts_[k_ + 1](1);
  // Path-tangential angle, eq. (10.55) p. 258 in Fossen2011.
  double alpha = atan2(yk_next - yk, xk_next - xk);
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
  double psi_des = alpha + chi_r;
  // if the next waypoint satisfy the switching criterion, k = k + 1
  double d =
    sqrt((xk_next - xk) * (xk_next - xk) + (yk_next - yk) * (yk_next - yk));
  int n = wpts_.size();
  if (d - s < R_ && k_ < n) {
    k_++;
  }
  // cross-track error differential equation
  double e_int_dot =
    delta_ * e /
    (delta_ * delta_ + (e + kappa_ * e_int_) * (e + kappa_ * e_int_));
  e_int_ += e_int_dot * dt_;
  return psi_des;
}
