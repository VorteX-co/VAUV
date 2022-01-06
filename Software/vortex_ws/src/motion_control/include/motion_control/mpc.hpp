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
#ifndef MOTION_CONTROL__MPC_HPP_
#define MOTION_CONTROL__MPC_HPP_

#include "plant.hpp"

class MPC
{
public:
  void set_params(
    const double & m, const double & volume, const int & T,
    const Vector6d & Ib, const Vector3d & r_cob,
    const Vector3d & r_cog, const Vector6d & Ma,
    const Vector6d & Dlinear, const Vector6d & Dquad,
    const Vector12d & Q, const Vector6d & R1, const Vector6d & R2,
    const Vector6d & tau_max, const Vector12d & state_max,
    const double & dt);
  // Control law calculation
  Vector6d action(
    Vector12d state, Vector12d desired_state,
    Vector6d feedforward_acc);

private:
  // From ENU to NED
  void to_SNAME(Vector12d & x);
  // Class for AUV dynamics
  Plant vehicle_;
  // Limits for the optimization variables [state, control]
  VectorXd vars_upperbounds_;
  VectorXd vars_lowerbounds_;
  Vector12i state_indices_;
  Vector6i control_indices_;
  // The state weighting matrix
  Vector12d Q_;
  // The control forces and control rate weighting Vectors
  // R1 for forces and torques, R2 for the rate of change of both
  Vector6d R1_;
  Vector6d R2_;
  // Prediction horizon length
  int T_;
  // Sampling time
  double dt_;
};

#endif  // MOTION_CONTROL__MPC_HPP_
