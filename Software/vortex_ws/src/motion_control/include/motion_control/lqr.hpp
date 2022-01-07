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

#ifndef MOTION_CONTROL__LQR_HPP_
#define MOTION_CONTROL__LQR_HPP_

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
  Vector6d action(Vector12d x, Vector12d xd, Vector6d feedforward_acc);

private:
  void to_SNAME(Vector12d & x);
  void saturate_control(Vector6d & tau);
  void saturate_error(Vector12d & delta);

  Plant vehicle_;
  CareSolver<12, 6> care_solver;
  // The control forces weighting matrix
  Matrix6d R_;
  // The state weighting matrix
  Matrix12d Q_;
  // Saturation values for control forces and torques
  Vector6d tau_max_;
  // Saturation values for tracking error
  Vector12d error_max_;
  Vector6d tau_;
};

#endif  // MOTION_CONTROL__LQR_HPP_
