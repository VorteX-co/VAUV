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

#include "lqr.hpp"
void LQR::set_params(
  const double & m, const double & volume, const Vector6d & Ib,
  const Vector3d & r_cob, const Vector3d & r_cog,
  const Vector6d & Ma, const Vector6d & Dlinear,
  const Vector6d & Dquad, const Vector12d & Q,
  const Vector6d & R, const Vector6d & tau_max,
  const Vector12d & error_max)
{
  vehicle_.initialize(m, volume, Ib, r_cob, r_cog, Ma, Dlinear, Dquad);
  tau_max_ = tau_max;
  error_max_ = error_max;
  Q_ = Q.asDiagonal();
  R_ = R.asDiagonal();
}
void LQR::saturate_control(Vector6d & tau)
{
  /* Saturating the control forces and moments at ± tau_max [N, N.m]
   */
  for (int i = 0; i <= 5; i++) {
    if (tau(i) > tau_max_(i)) {
      tau(i) = tau_max_(i);
    } else if (tau(i) < -tau_max_(i)) {
      tau(i) = -tau_max_(i);
    }
  }
}
void LQR::saturate_error(Vector12d & delta)
{
  /* Saturating the state errors ± error_max
   */
  for (int i = 0; i <= 11; i++) {
    if (delta(i) > error_max_(i)) {
      delta(i) = error_max_(i);
    } else if (delta(i) < -error_max_(i)) {
      delta(i) = -error_max_(i);
    }
  }
}
// =========================================================================
void LQR::to_SNAME(Vector12d & x)
{
  /* from ENU to NED.
   */
  x(1) *= -1;
  x(2) *= -1;
  x(4) *= -1;
  x(5) *= -1;
  x(7) *= -1;
  x(8) *= -1;
  x(10) *= -1;
  x(11) *= -1;
}
Vector6d LQR::action(Vector12d x, Vector12d xd, Vector6d feedforward_acc)
{
  to_SNAME(x);
  to_SNAME(xd);
  // MatrixXd A, B;
  Matrix12d A;
  Eigen::Matrix<double, 12, 6> B;
  // Calculating the Jacobian based linearization of the dynamics
  vehicle_.linearize(x, tau_, A, B);
  // Solving the Continous Algebraic Riccati Equation (CARE)
  Matrix12d P = Matrix12d::Zero();
  care_solver.solve(P, A, B, Q_, R_);
  // The optimal feedback gain matrix
  Eigen::Matrix<double, 6, 12> K = -R_.inverse() * B.transpose() * P;
  // LQR control law
  Vector6d tau = K * (x - xd);
  saturate_control(tau);
  tau_ = tau;
  return tau;
}
