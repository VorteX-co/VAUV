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

#include "lqr.hpp"

// =========================================================================
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
// =========================================================================
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
// =========================================================================
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
// =========================================================================
Vector6d LQR::action(Vector12d x, Vector12d xd, Vector6d feedforward_acc)
{
  to_SNAME(x);
  to_SNAME(xd);
  Eigen::Matrix<double, 12, 2> logging;
  logging << x, xd;
  std::cout << "########### Logging #############" << std::endl;
  std::cout << logging << std::endl;
  // MatrixXd A, B;
  Matrix12d A;
  Eigen::Matrix<double, 12, 6> B;
  // Calculating the Jacobian based linearization of the dynamics
  vehicle_.linearize(x, A, B);
  // Solving the Continous Algebraic Riccati Equation (CARE)
  Matrix12d P = Matrix12d::Zero();
  care_solver.solve(P, A, B, Q_, R_);
  // The optimal feedback gain matrix
  Eigen::Matrix<double, 6, 12> K = -R_.inverse() * B.transpose() * P;
  // LQR control law
  Vector12d error = (x - xd);
  saturate_error(error);
  Vector6d tau = K * error;
  saturate_control(tau);
  std::cout << "########### Tau #############" << std::endl;
  std::cout << tau << std::endl;
  return tau;
}
// =========================================================================
Vector12d LQR::simulation_rk4(const Vector12d & x0, const Vector6d & u)
{
  /*
   * Simulating the control forces on nonlinear vehicle model
   *  Runge-Kutta (4th-order) method for integrating the equations of motion..
   */
  Vector12d x;
  Vector12d k1 = 0.05 * vehicle_.nonlinear_update(x0, u);
  x = x0 + 0.5 * k1;
  Vector12d k2 = 0.05 * vehicle_.nonlinear_update(x, u);
  x = x0 + 0.5 * k2;
  Vector12d k3 = 0.05 * vehicle_.nonlinear_update(x, u);
  x = x0 + k3;
  Vector12d k4 = 0.05 * vehicle_.nonlinear_update(x, u);
  Vector12d xnext = x0 + (k1 + 2 * (k2 + k3) + k4) / 6;
  return xnext;
}
// =========================================================================
Vector12d LQR::simulation_euler(const Vector12d & x0, const Vector6d & u)
{
  /*
   * Simulating the control forces on nonlinear vehicle model
   *  Euler method for integrating the equations of motion.
   */
  Vector12d xnext = x0 + 0.05 * vehicle_.nonlinear_update(x0, u);
  return xnext;
}
