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
#ifndef MOTION_CONTROL__IPOPT_MPC_PROBLEM_HPP_
#define MOTION_CONTROL__IPOPT_MPC_PROBLEM_HPP_

#include <cppad/example/cppad_eigen.hpp>
#include "plant.hpp"

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using ADscalar = CppAD::AD<double>;

/******************************************************************************************************************
 * Class for formulating the MPC tracking control problem
 * as nonlinear programming for ipopt::solver.
 * When calling ipopt::solve(xi, xl, xu, gl, gu, problem, solution) ipopt uses
 * this class for computing the objective value at each iteration in the
 * optimization process. The class must  supports the syntax problem::ADvector
 * CppAD::AD<double> based data type. problem(fg, x)           Call for
 * calculating fg (cost and constraints) given the optimization variables x.
 * See, https://coin-or.github.io/CppAD/doc/ipopt_solve.htm
 * ******************************************************************************************************************/
class MPCProblem
{
public:
  using ADvector = Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>;
  MPCProblem(
    const Plant & AUV, const double & dt, const int & T,
    const VectorXd & desired_state, const VectorXd & Q,
    const VectorXd & R1, const VectorXd & R2,
    const Vector12i & state_indices, const Vector6i & control_indices)
  {
    AUV_ = AUV;
    T_ = T;
    dt_ = dt;
    desired_state_ = desired_state.template cast<CppAD::AD<double>>();
    Q_ = Q.template cast<ADscalar>();
    R1_ = R1.template cast<ADscalar>();
    R2_ = R2.template cast<ADscalar>();
    state_indices_ = state_indices;
    control_indices_ = control_indices;
  }
  void operator()(ADvector & fg, const ADvector & vars)
  {
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    for (int i = 0; i < T_; i++) {
      // Error cost
      ADvector error =
        vars(state_indices_ + Vector12i::Ones() * i) - desired_state_;
      fg(0) += error.cwiseProduct(Q_).squaredNorm();
    }
    for (int i = 0; i < T_ - 1; i++) {
      // Actuators usage cost
      fg(0) += vars(control_indices_ + Vector6i::Ones() * i)
        .cwiseProduct(R1_)
        .squaredNorm();
    }
    for (int i = 0; i < T_ - 2; i++) {
      // Actuators usage rate cost
      ADvector dtau = vars(control_indices_ + Vector6i::Ones() * (i + 1)) -
        vars(control_indices_ + Vector6i::Ones() * i);
      fg(0) += dtau.cwiseProduct(R2_).squaredNorm();
    }
    // Model constraints.
    // The the initial constraints
    fg(state_indices_ + Vector12i::Ones()) = vars(state_indices_);
    for (int i = 1; i < T_; i++) {
      // The state at time t.
      ADvector state0 = vars(state_indices_ + Vector12i::Ones() * (i - 1));
      // The control at time t.
      ADvector control0 = vars(control_indices_ + Vector6i::Ones() * (i - 1));
      // The state at time t+1.
      ADvector state1 = vars(state_indices_ + Vector12i::Ones() * (i));
      // AUV motion model f(x,u)
      ADvector dx = AUV_.nonlinear_update(state0, control0);
      // Model constraints. x_k+1 = f(x_k, u_k)
      fg(state_indices_ + Vector12i::Ones() * (1 + i)) =
        state1 - (state0 + dx * dt_);
    }
  }

private:
  // Class for computing AUV dynamics with CppAD::AD based data types
  Plant AUV_;
  // Prediction horizon
  int T_;
  // Sampling time
  ADscalar dt_;
  // Desired state, used for calculating the cost.
  ADvector desired_state_;
  // Error weighting vector
  ADvector Q_;
  // Body effort weighting vector
  ADvector R1_;
  // Body effort rate weighting vector
  ADvector R2_;
  // Indices for the initial state from large vector containts T prediciton
  // horizon * 12 state. ADvector::Ones + state_indices gives the next time step
  // state indices.
  Vector12i state_indices_;
  Vector6i control_indices_;
};
#endif  // MOTION_CONTROL__IPOPT_MPC_PROBLEM_HPP_
