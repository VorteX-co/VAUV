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
#ifndef TRAJECTORY_TRACKING_LQR__LQR_HPP_
#define TRAJECTORY_TRACKING_LQR__LQR_HPP_
#include "lin_alg_tools/care.h"
#include "lin_alg_tools/schur.h"
#include "plant.hpp"

class LQR
{
public:
  void set_params(
    const double & m, const double & volume, const Vector6d & Ib,
    const Vector3d & r_cob, const Vector3d & r_cog,
    const Vector6d & Ma, const Vector6d & Dlinear,
    const Vector6d & Dquad, const Vector12d & Q, const Vector6d & R,
    const Vector6d & tau_max, const Vector12d & error_max);

  // LQR control action  given the current state and desired state
  Vector6d action(Vector12d x, Vector12d xd, Vector6d feedforward_acc);

  // Method of testing the LQR control action on nonlinear vehicle model
  // @params (inital state and control vector)
  Vector12d simulation_rk4(const Vector12d & x0, const Vector6d & u);
  Vector12d simulation_euler(const Vector12d & x0, const Vector6d & u);

private:
  void to_SNAME(Vector12d & x);
  void saturate_control(Vector6d & tau);
  void saturate_error(Vector12d & delta);
  // Class for kinematics and dynamics computation
  // as well as linearizing these computed functions
  Plant vehicle_;

  // Continuous algebraic Riccati equation (Care) solver [12 states, 6 controls]
  CareSolver<12, 6> care_solver;
  // The control forces weighting matrix
  Matrix6d R_;
  // The state weighting matrix
  Matrix12d Q_;
  // Saturation values for control forces and torques
  Vector6d tau_max_;
  // Saturation values for tracking error
  Vector12d error_max_;
};
#endif  // TRAJECTORY_TRACKING_LQR__LQR_HPP_
