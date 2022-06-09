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

#include "mpc.hpp"
#include <cppad/ipopt/solve.hpp>
#include <cmath>
#include <string>
#include "ipopt_mpc_problem.hpp"

void MPC::set_params(
  const double & m, const double & volume, const int & T,
  const Vector6d & Ib, const Vector3d & r_cob,
  const Vector3d & r_cog, const Vector6d & Ma,
  const Vector6d & Dlinear, const Vector6d & Dquad,
  const Vector12d & Q, const Vector6d & R1, const Vector6d & R2,
  const Vector6d & tau_max,
  const double & dt)
{
  vehicle_.initialize(m, volume, Ib, r_cob, r_cog, Ma, Dlinear, Dquad);
  tau_max_ = tau_max;
  state_indices_ << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
  state_indices_ *= T;
  int control_start = state_indices_(11) + T;
  control_indices_ << 0, 1, 2, 3, 4, 5;
  control_indices_ *= T - 1;
  control_indices_ += Vector6i::Ones() * control_start;
  T_ = T;
  Q_ = Q;
  R1_ = R1;
  R2_ = R2;
  dt_ = dt;
}
// =========================================================================
void MPC::to_SNAME(Vector12d & x)
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
Vector6d MPC::action(
  Vector12d state, Vector12d desired_state,
  Vector6d feedforward_acc)
{
  to_SNAME(state);
  to_SNAME(desired_state);
  // Optimization variables (state, control)
  VectorXd vars(T_ * 12 + (T_ - 1) * 6);
  vars.setZero();
  // initial state
  vars(state_indices_) = state;
  VectorXd vars_upperbounds(T_ * 12 + (T_ - 1) * 6);
  for (size_t i = 0; i < T_; i++) {
    vars_upperbounds(state_indices_ + Vector12i::Ones() * i) = 1.e9 * Vector12d::Ones();
  }
  for (size_t i = 0; i < T_ - 1; i++) {
    vars_upperbounds(control_indices_ + Vector6i::Ones() * i) = tau_max_;
  }
  VectorXd vars_lowerbounds(T_ * 12 + (T_ - 1) * 6);
  vars_lowerbounds = -1 * vars_upperbounds;
  // Lower and upper limits for constraints
  VectorXd constraints_upperbounds(T_ * 12);
  VectorXd constraints_lowerbounds(T_ * 12);
  constraints_upperbounds.setZero();
  // Initial state
  constraints_upperbounds(state_indices_) = state;
  constraints_lowerbounds.setZero();
  constraints_lowerbounds(state_indices_) = state;

  // Object that computes objective and constraints
  MPCProblem problem(
    vehicle_, dt_, T_, Q_, R1_, R2_, desired_state, state_indices_, control_indices_,
    error_indices_);
  // place to return solution
  CppAD::ipopt::solve_result<VectorXd> solution;

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // solve the problem
  CppAD::ipopt::solve<VectorXd, MPCProblem>(
    options, vars, vars_lowerbounds, vars_upperbounds,
    constraints_lowerbounds, constraints_upperbounds, problem, solution);

  VectorXd tau = solution.x(control_indices_);
  return tau;
}
