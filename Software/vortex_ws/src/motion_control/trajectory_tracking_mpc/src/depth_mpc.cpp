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
#include "depth_mpc.hpp"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <map>
#include <vector>
#include <string>
#include "eigen3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

class FG_eval
{
public:
  Eigen::Vector2d reference_state;
  double reference_acc;
  std::map<string, double> depth_params;
  double _w_ze, _w_thetae, _w_we, _w_Fz, _w_My, _w_Fz_d, _w_My_d;

  Eigen::Vector2d _M;
  Eigen::Vector2d _QD;
  Eigen::Vector2d _LD;
  double _WEIGHT;
  double _B;

  FG_eval(
    Eigen::Vector2d reference_state, double reference_acc,
    const std::map<string, double> & depth_params)
  {
    this->reference_state = reference_state;
    this->reference_acc = reference_acc;
    this->depth_params = depth_params;
    _w_ze = this->depth_params["w_ze"];
    _w_we = this->depth_params["w_we"];
    _w_thetae = this->depth_params["w_thetae"];
    _w_Fz = this->depth_params["w_Fz"];
    _w_Fz_d = this->depth_params["w_Fz_dot"];
    _w_My = this->depth_params["w_My"];
    _w_My_d = this->depth_params["w_My_dot"];

    _WEIGHT = this->depth_params["weight"];
    _B = this->depth_params["buoyancy"];
    _M << this->depth_params["Mz"], this->depth_params["Mtheta"];
    _QD << -this->depth_params["QDz"], -this->depth_params["QDtheta"];
    _LD << -this->depth_params["LDz"], -this->depth_params["LDtheta"];
  }

  typedef CPPAD_TESTVECTOR (AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector & fg, const ADvector & vars)
  {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    size_t N = depth_params["N"];
    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    size_t _z_start = 0;
    size_t _theta_start = _z_start + N;
    size_t _w_start = _theta_start + N;
    size_t _q_start = _w_start + N;
    size_t _ze_start = _q_start + N;
    size_t _we_start = _ze_start + N;
    size_t _thetae_start = _we_start + N;
    size_t _Fz_start = _thetae_start + N;
    size_t _My_start = _Fz_start + N - 1;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; ++t) {
      fg[0] += _w_ze * CppAD::pow(vars[_ze_start + t], 2);
      fg[0] += _w_we * CppAD::pow(vars[_we_start + t], 2);
      fg[0] += _w_thetae * CppAD::pow(vars[_thetae_start + t], 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += _w_Fz * CppAD::pow(vars[_Fz_start + t], 2);
      fg[0] += _w_My * CppAD::pow(vars[_My_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += _w_Fz_d *
        CppAD::pow(vars[_Fz_start + t + 1] - vars[_Fz_start + t], 2);
      fg[0] += _w_My_d *
        CppAD::pow(vars[_My_start + t + 1] - vars[_My_start + t], 2);
    }
    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + _z_start] = vars[_z_start];
    fg[1 + _theta_start] = vars[_theta_start];
    fg[1 + _w_start] = vars[_w_start];
    fg[1 + _q_start] = vars[_q_start];
    fg[1 + _ze_start] = vars[_ze_start];
    fg[1 + _we_start] = vars[_we_start];
    fg[1 + _thetae_start] = vars[_thetae_start];

    // The rest of the constraints
    for (int t = 1; t < N; ++t) {
      // The state at time t+1 .
      AD<double> z1 = vars[_z_start + t];
      AD<double> theta1 = vars[_theta_start + t];
      AD<double> w1 = vars[_w_start + t];
      AD<double> q1 = vars[_q_start + t];
      AD<double> ze1 = vars[_ze_start + t];
      AD<double> thetae1 = vars[_thetae_start + t];
      AD<double> we1 = vars[_we_start + t];

      // The state at time t.
      AD<double> z0 = vars[_z_start + t - 1];
      AD<double> theta0 = vars[_theta_start + t - 1];
      AD<double> w0 = vars[_w_start + t - 1];
      AD<double> q0 = vars[_q_start + t - 1];
      AD<double> ze0 = vars[_ze_start + t - 1];
      AD<double> thetae0 = vars[_thetae_start + t - 1];
      AD<double> we0 = vars[_we_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> Fz0 = vars[_Fz_start + t - 1];
      AD<double> My0 = vars[_My_start + t - 1];

      // Dynamics
      AD<double> Wdot =
        1 / _M(0) *
        (_WEIGHT - _B + _QD(0) * w0 + _LD(0) * CppAD::abs(w0) * w0 + Fz0);
      AD<double> Qdot =
        1 / _M(1) * (_QD(1) * q0 + _LD(1) * CppAD::abs(q0) * q0 + My0);

      AD<double> z_des = reference_state(0);
      AD<double> w_des = reference_state(1);
      AD<double> theta_des = 0.0;
      AD<double> Wdot_des = reference_acc;

      AD<double> ze_nominal = (z0 - z_des);
      AD<double> thetae_nominal = -1 * (theta0 - theta_des);
      AD<double> we_nominal = w0 - w_des;

      AD<double> ze_dot = we_nominal;
      AD<double> thetae_dot = -q0;
      AD<double> we_dot = Wdot - Wdot_des;

      double dt = depth_params["dt"];

      /////////////////////////////////////////////////////////////////////////
      /// State  Dynamics
      fg[1 + _z_start + t] = z1 - (z0 + w0 * dt);
      fg[1 + _theta_start + t] = theta1 - (theta0 + q0 * dt);
      fg[1 + _w_start + t] = w1 - (w0 + Wdot * dt);
      //////////////////////////////////////////////////////////////////////
      /// Error Dynamics
      fg[1 + _ze_start + t] = ze1 - (ze_nominal + ze_dot * dt);
      fg[1 + _thetae_start + t] = thetae1 - (thetae_nominal + thetae_dot * dt);
      fg[1 + _we_start + t] = we1 - (we_nominal + we_dot * dt);
    }
  }
};

//
// MPC class definition
//
void Depth::MPC::set_params(const std::map<string, double> & params)
{
  depth_params = params;
}

Depth::MPC::MPC() {}
Depth::MPC::~MPC() {}

std::vector<double> Depth::MPC::Solve(
  const Eigen::VectorXd & x0,
  const Eigen::Vector2d & reference_state,
  const double & reference_acc)
{
  typedef CPPAD_TESTVECTOR (double) Dvector;

  double z = x0[0];
  double theta = x0[1];
  double w = x0[2];
  double q = x0[3];
  double ze = x0[4];
  double thetae = x0[5];
  double we = x0[6];

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  size_t N = depth_params["N"];
  size_t _z_start = 0;
  size_t _theta_start = _z_start + N;
  size_t _w_start = _theta_start + N;
  size_t _q_start = _w_start + N;
  size_t _ze_start = _q_start + N;
  size_t _we_start = _ze_start + N;
  size_t _thetae_start = _we_start + N;
  size_t _Fz_start = _thetae_start + N;
  size_t _My_start = _Fz_start + N - 1;
  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 7 + (N - 0) * 2;
  // Number of constraints
  size_t n_constraints = N * 7 + 2 * N;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[_z_start] = z;
  vars[_theta_start] = theta;
  vars[_w_start] = w;
  vars[_q_start] = q;
  vars[_ze_start] = ze;
  vars[_thetae_start] = thetae;
  vars[_we_start] = we;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < _theta_start; i++) {
    vars_lowerbound[i] = -depth_params["max_z"];
    vars_upperbound[i] = 0.0;
  }
  for (int i = _theta_start; i < _w_start; i++) {
    vars_lowerbound[i] = -depth_params["max_theta"] + 0.001;
    vars_upperbound[i] = depth_params["max_theta"] - 0.001;
  }
  for (int i = _w_start; i < _q_start; i++) {
    vars_lowerbound[i] = -depth_params["max_w"];
    vars_upperbound[i] = depth_params["max_w"];
  }
  for (int i = _q_start; i < _ze_start; i++) {
    vars_lowerbound[i] = -depth_params["max_q"];
    vars_upperbound[i] = depth_params["max_q"];
  }
  for (int i = _ze_start; i < _thetae_start; i++) {
    vars_lowerbound[i] = -depth_params["max_z"];
    vars_upperbound[i] = depth_params["max_z"];
  }
  for (int i = _thetae_start; i < _we_start; i++) {
    vars_lowerbound[i] = -depth_params["max_theta"] + 0.001;
    vars_upperbound[i] = depth_params["max_theta"] - 0.001;
  }
  for (int i = _we_start; i < _Fz_start; i++) {
    vars_lowerbound[i] = -depth_params["max_w"];
    vars_upperbound[i] = depth_params["max_w"];
  }

  vars_lowerbound[_Fz_start] = prev_Fz;
  vars_upperbound[_Fz_start] = prev_Fz;

  for (int i = _Fz_start; i < _My_start; i++) {
    vars_lowerbound[i] = -depth_params["max_Fz"];
    vars_upperbound[i] = depth_params["max_Fz"];
  }

  vars_lowerbound[_My_start] = prev_My;
  vars_upperbound[_My_start] = prev_My;

  for (int i = _My_start; i < n_vars; i++) {
    vars_lowerbound[i] = -depth_params["max_My"];
    vars_upperbound[i] = depth_params["max_My"];
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[_z_start] = z;
  constraints_lowerbound[_theta_start] = theta;
  constraints_lowerbound[_w_start] = w;
  constraints_lowerbound[_q_start] = q;
  constraints_lowerbound[_ze_start] = ze;
  constraints_lowerbound[_thetae_start] = thetae;
  constraints_lowerbound[_we_start] = we;

  constraints_upperbound[_z_start] = z;
  constraints_upperbound[_theta_start] = theta;
  constraints_upperbound[_w_start] = w;
  constraints_upperbound[_q_start] = q;
  constraints_upperbound[_ze_start] = ze;
  constraints_upperbound[_thetae_start] = thetae;
  constraints_upperbound[_we_start] = we;

  // Object that computes objective and constraints
  FG_eval fg_eval(reference_state, reference_acc, depth_params);
  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  std::cout << "OK? " << ok << std::endl;

  prev_Fz = solution.x[_Fz_start + 1];
  prev_My = solution.x[_My_start + 1];

  return {solution.x[_Fz_start], solution.x[_My_start]};
}
