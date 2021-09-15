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
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
void LOS::setpoint(const Eigen::Vector2d & wp)
{
  prev_goal = goal;
  goal = wp;
}
void LOS::update_state(
  const Eigen::Vector3d & pose,
  const Eigen::Vector3d & vel)
{
  pose_state = pose;
  vel_state = vel;
}
void LOS::InCircle()
{
  double dx = goal(0) - pose_state(0);
  double dy = goal(1) - pose_state(1);
  // Check if the vehicle within the sphere of acceptance
  inCircle = std::sqrt(dx * dx + dy * dy) < R;
}
void LOS::calculate_reference()
{
  double dx = goal(0) - prev_goal(0);
  double dy = goal(1) - prev_goal(1);
  // Path-tangential angle, eq. (10.55) p. 258 in Fossen2011.
  double alpha = std::atan2(dy, dx);
  // Alonge-track and cross-track errors, eq. (10.58, 10.59) p. 258 in
  // Fossen2011.
  double s = (pose_state(0) - prev_goal(0)) * cos(alpha) +
    (pose_state(1) - prev_goal(1)) * sin(alpha);
  double e = -(pose_state(0) - prev_goal(0)) * sin(alpha) +
    (pose_state(1) - prev_goal(1)) * cos(alpha);
  // Velocity-path relative angle, eq. (10.74) p. 261 in Fossen2011.
  double chi_r = atan(-e / delta);
  // Desired course angle, eq. (10.72) p. 261 in Fossen2011.
  double chi_des = alpha + chi_r;
  // Total speed
  double Ut = std::sqrt(vel_state(0) * vel_state(0) +
      vel_state(1) * vel_state(1) + 1e-7);
  // Side-slip (drift) angle
  double beta = std::asin(vel_state(1) / Ut);
  // Storing the previous values for smooth reference calculation
  yaw_ref_prev_prev = yaw_ref_prev;
  yaw_ref_prev = yaw_ref;
  // Desired yaw angle, eq. (10.83) p. 263 in Fossen2011.
  // yaw_ref = chi_des - beta;
  // Assuming that beta is very small ~= 0
  yaw_ref = chi_des;
  double psie = pose_state(2) - yaw_ref;
  if (psie > M_PI) {
    yaw_ref += 2 * M_PI;
  }
  if (psie < -M_PI) {
    yaw_ref -= 2 * M_PI;
  }
  smooth_reference();

  // Reference yaw angular rate
  r_ref = (yaw_des - yaw_des_prev) / dt;  // - 0.1 * psie;
  rd_prev = rd;
  // Low-pass filter for smooth output signal
  rd = rd_prev - 0.35 * (rd_prev - r_ref);
  rd = std::min(std::max(rd, -0.25), 0.25);
  // Reference yaw angular acceleration
  rdd = (rd - rd_prev) / dt;
  rdd = std::min(std::max(rdd, -0.1), 0.1);
}
void LOS::smooth_reference()
{
  // Reference model by the dynamics of mass-spring-damper system
  // The coefficients is based on the work from (Kristoffer Solberg's master
  // thesis) p.119:120
  yaw_des_prev_prev = yaw_des_prev;
  yaw_des_prev = yaw_des;
  yaw_des = 2.9581e-04 * yaw_ref + 5.9161e-04 * yaw_ref_prev +
    2.9581e-04 * yaw_ref_prev_prev - -1.9312 * yaw_des_prev -
    0.9324 * yaw_des_prev_prev;
}
