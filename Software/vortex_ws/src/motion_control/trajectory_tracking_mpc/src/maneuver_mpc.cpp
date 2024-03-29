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
#include "maneuver_mpc.hpp"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include <string>
#include <map>
#include "eigen3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

/**
 * @brief The FG_eval class computes the ojectives and constraints
 * required by the ipopt solver
 */

class FG_eval
{
public:
  Eigen::VectorXd reference_state;
  Eigen::Vector3d reference_acc;
  std::map<string, double> maneuver_params;
  double _w_xe, _w_ye, _w_psie, _w_ue, _w_ve, _w_re, _w_Fx, _w_Fy, _w_Mz,
    _w_Fx_d, _w_Fy_d, _w_Mz_d;
  Eigen::Vector3d _M;
  Eigen::Vector3d _QD;
  Eigen::Vector3d _LD;
  /**
   * @brief constructor for initializing the local vaiables
   * @param reference_state
   * @param reference_acc
   * @param maneuver_params
   */
  FG_eval(
    const Eigen::VectorXd & reference_state,
    const Eigen::Vector3d & reference_acc,
    const std::map<string, double> & maneuver_params)
  {
    this->maneuver_params = maneuver_params;
    this->reference_state = reference_state;
    this->reference_acc = reference_acc;
    _w_xe = this->maneuver_params["w_xe"];
    _w_ye = this->maneuver_params["w_ye"];
    _w_psie = this->maneuver_params["w_psie"];
    _w_ue = this->maneuver_params["w_ue"];
    _w_ve = this->maneuver_params["w_ve"];
    _w_re = this->maneuver_params["w_re"];
    _w_Fx = this->maneuver_params["w_Fx"];
    _w_Fx_d = this->maneuver_params["w_Fx_dot"];
    _w_Fy = this->maneuver_params["w_Fy"];
    _w_Fy_d = this->maneuver_params["w_Fy_dot"];
    _w_Mz = this->maneuver_params["w_Mz"];
    _w_Mz_d = this->maneuver_params["w_Mz_dot"];

    _M << this->maneuver_params["Mx"], this->maneuver_params["My"],
      this->maneuver_params["Mpsi"];
    _QD << this->maneuver_params["QDx"], this->maneuver_params["QDy"],
      this->maneuver_params["QDpsi"];
    _LD << this->maneuver_params["LDx"], this->maneuver_params["LDy"],
      this->maneuver_params["LDpsi"];
  }

  typedef CPPAD_TESTVECTOR (AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector & fg, const ADvector & vars)
  {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    // Prediction horizon
    size_t N = maneuver_params["N"];
    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    size_t _x_start = 0;
    size_t _y_start = _x_start + N;
    size_t _psi_start = _y_start + N;
    size_t _u_start = _psi_start + N;
    size_t _v_start = _u_start + N;
    size_t _r_start = _v_start + N;
    size_t _xe_start = _r_start + N;
    size_t _ye_start = _xe_start + N;
    size_t _psie_start = _ye_start + N;
    size_t _ue_start = _psie_start + N;
    size_t _ve_start = _ue_start + N;
    size_t _re_start = _ve_start + N;
    size_t _Fx_start = _re_start + N;
    size_t _Fy_start = _Fx_start + N;
    size_t _Mz_start = _Fy_start + N - 1;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; ++t) {
      fg[0] += _w_xe * CppAD::pow(vars[_xe_start + t], 2);
      fg[0] += _w_ye * CppAD::pow(vars[_ye_start + t], 2);
      fg[0] += _w_psie * CppAD::pow(vars[_psie_start + t], 2);
      fg[0] += _w_ue * CppAD::pow(vars[_ue_start + t], 2);
      fg[0] += _w_ve * CppAD::pow(vars[_ve_start + t], 2);
      fg[0] += _w_re * CppAD::pow(vars[_re_start + t], 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; ++t) {
      fg[0] += _w_Fx * CppAD::pow(vars[_Fx_start + t], 2);
      fg[0] += _w_Fy * CppAD::pow(vars[_Fy_start + t], 2);
      fg[0] += _w_Mz * CppAD::pow(vars[_Mz_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; ++t) {
      fg[0] += _w_Fx_d *
        CppAD::pow(vars[_Fx_start + t + 1] - vars[_Fx_start + t], 2);
      fg[0] += _w_Fy_d *
        CppAD::pow(vars[_Fy_start + t + 1] - vars[_Fy_start + t], 2);
      fg[0] += _w_Mz_d *
        CppAD::pow(vars[_Mz_start + t + 1] - vars[_Mz_start + t], 2);
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
    fg[1 + _x_start] = vars[_x_start];
    fg[1 + _y_start] = vars[_y_start];
    fg[1 + _psi_start] = vars[_psi_start];
    fg[1 + _u_start] = vars[_u_start];
    fg[1 + _v_start] = vars[_v_start];
    fg[1 + _r_start] = vars[_r_start];
    fg[1 + _xe_start] = vars[_xe_start];
    fg[1 + _ye_start] = vars[_ye_start];
    fg[1 + _psie_start] = vars[_psie_start];
    fg[1 + _ue_start] = vars[_ue_start];
    fg[1 + _ve_start] = vars[_ve_start];
    fg[1 + _re_start] = vars[_re_start];

    // The rest of the constraints
    for (int t = 1; t < N; ++t) {
      // The state at time t+1 .
      AD<double> x1 = vars[_x_start + t];
      AD<double> y1 = vars[_y_start + t];
      AD<double> psi1 = vars[_psi_start + t];
      AD<double> u1 = vars[_u_start + t];
      AD<double> v1 = vars[_v_start + t];
      AD<double> r1 = vars[_r_start + t];
      AD<double> xe1 = vars[_xe_start + t];
      AD<double> ye1 = vars[_ye_start + t];
      AD<double> psie1 = vars[_psie_start + t];
      AD<double> ue1 = vars[_ue_start + t];
      AD<double> ve1 = vars[_ve_start + t];
      AD<double> re1 = vars[_re_start + t];

      // The state at time t.
      AD<double> x0 = vars[_x_start + t - 1];
      AD<double> y0 = vars[_y_start + t - 1];
      AD<double> psi0 = vars[_psi_start + t - 1];
      AD<double> u0 = vars[_u_start + t - 1];
      AD<double> v0 = vars[_v_start + t - 1];
      AD<double> r0 = vars[_r_start + t - 1];
      AD<double> xe0 = vars[_xe_start + t - 1];
      AD<double> ye0 = vars[_ye_start + t - 1];
      AD<double> psie0 = vars[_psie_start + t - 1];
      AD<double> ue0 = vars[_ue_start + t - 1];
      AD<double> ve0 = vars[_ve_start + t - 1];
      AD<double> re0 = vars[_re_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> Fx0 = vars[_Fx_start + t - 1];
      AD<double> Fy0 = vars[_Fy_start + t - 1];
      AD<double> Mz0 = vars[_Mz_start + t - 1];
      //////////////////////////////////////////////////////////////////////////////////
      // Vehicle dynamics induced from actuation
      AD<double> Udot =
        1 / (_M(0)) *
        (_M(1) * v0 * r0 - _LD(0) * u0 - _QD(0) * CppAD::abs(u0) * u0 + Fx0);
      AD<double> Vdot = 1 / (_M(1)) *
        (-1 * _M(0) * u0 * r0 - _LD(1) * v0 -
        _QD(1) * CppAD::abs(v0) * v0 + Fy0);
      AD<double> Rdot = 1 / _M(2) *
        ((_M(0) - _M(1)) * u0 * v0 - _LD(2) * r0 -
        _QD(2) * CppAD::abs(r0) * r0 + Mz0);
      // desired state for error-dynamics calculation
      AD<double> x_des = reference_state(0);
      AD<double> y_des = reference_state(1);
      AD<double> psi_des = reference_state(2);
      AD<double> u_des = reference_state(3);
      AD<double> v_des = reference_state(4);
      AD<double> r_des = reference_state(5);
      // Feedforward acceleration for reducing the effects of  disturbances
      AD<double> Udot_des = reference_acc(0);
      AD<double> Vdot_des = reference_acc(1);
      AD<double> Rdot_des = reference_acc(2);
      // All the following errors definition is based on the work from
      // Chao Shen, 2018 dissertation
      // Nominal positional error definition, eq. (3.8) p38
      AD<double> xe_nominal =
        (x0 - x_des) * -CppAD::cos(psi0) - (y0 - y_des) * CppAD::sin(psi0);
      AD<double> ye_nominal =
        (x0 - x_des) * CppAD::sin(psi0) - (y0 - y_des) * CppAD::cos(psi0);
      AD<double> psie_nominal = (psi_des - psi0);
      // Nominal velocity error definition,  eq. (5.41) p99
      AD<double> ue_nominal = u0 - u_des * CppAD::cos(psie_nominal) +
        v_des * CppAD::sin(psie_nominal);
      AD<double> ve_nominal = v0 - u_des * CppAD::sin(psie_nominal) -
        v_des * CppAD::cos(psie_nominal);
      AD<double> re_nominal = (r0 - r_des);
      // How the positional error evolves with time, eq.(3.9) p38
      AD<double> xe_dot =
        ye_nominal * r_des - ue_nominal + ye_nominal * re_nominal;
      AD<double> ye_dot =
        -xe_nominal * r_des - ve_nominal - xe_nominal * re_nominal;
      AD<double> psie_dot = -re_nominal;
      // How the velocity error evolves with time, eq.(3.12) p38
      AD<double> ue_dot =
        (_M(1) / _M(0)) *
        (ve_nominal + u_des * CppAD::sin(psie_nominal) +
        v_des * CppAD::cos(psie_nominal)) *
        (re_nominal + r_des) -
        _LD(0) / _M(0) *
        (ue_nominal + u_des * CppAD::cos(psie_nominal) -
        v_des * CppAD::sin(psie_nominal)) -
        _QD(0) / _M(0) *
        (ue_nominal + u_des * CppAD::cos(psie_nominal) -
        v_des * CppAD::sin(psie_nominal)) *
        CppAD::abs(ue_nominal + u_des * CppAD::cos(psie_nominal) -
          v_des * CppAD::sin(psie_nominal)) -
        Udot_des * CppAD::cos(psie_nominal) +
        Vdot_des * CppAD::sin(psie_nominal) -
        u_des * CppAD::sin(psie_nominal) * re_nominal -
        v_des * CppAD::cos(psie_nominal) * re_nominal + Fx0 / _M(0);

      AD<double> ve_dot =
        (-_M(0) / _M(1)) *
        (ue_nominal + u_des * CppAD::cos(psie_nominal) -
        v_des * CppAD::sin(psie_nominal)) *
        (re_nominal + r_des) -
        _LD(1) / _M(1) *
        (ve_nominal + u_des * CppAD::sin(psie_nominal) +
        v_des * CppAD::cos(psie_nominal)) -
        _QD(1) / _M(1) *
        (ve_nominal + u_des * CppAD::sin(psie_nominal) +
        v_des * CppAD::cos(psie_nominal)) *
        CppAD::abs(ve_nominal + u_des * CppAD::sin(psie_nominal) +
          v_des * CppAD::cos(psie_nominal)) -
        Udot_des * CppAD::sin(psie_nominal) -
        Vdot_des * CppAD::cos(psie_nominal) +
        u_des * CppAD::cos(psie_nominal) * re_nominal -
        v_des * CppAD::sin(psie_nominal) * re_nominal + Fy0 / _M(1);

      AD<double> re_dot = (_M(0) - _M(1)) / _M(2) *
        (ue_nominal + u_des * CppAD::cos(psie_nominal) -
        v_des * CppAD::sin(psie_nominal)) *
        (ve_nominal + u_des * CppAD::sin(psie_nominal) +
        v_des * CppAD::cos(psie_nominal)) -
        _LD(2) / _M(2) * (re_nominal + r_des) -
        _QD(2) / _M(2) * (re_nominal + r_des) *
        CppAD::abs(re_nominal + r_des) -
        Rdot_des + Mz0 / _M(2);
      // Constraints evolution after dt
      // Discretization interval
      double dt = maneuver_params["dt"];
      // AUV state
      fg[1 + _x_start + t] =
        x1 - (x0 + u0 * CppAD::cos(psi0) * dt - v0 * CppAD::sin(psi0) * dt);
      fg[1 + _y_start + t] =
        y1 - (y0 + u0 * CppAD::sin(psi0) * dt + v0 * CppAD::cos(psi0) * dt);
      fg[1 + _psi_start + t] = psi1 - (psi0 + r0 * dt);
      fg[1 + _u_start + t] = u1 - (u0 + Udot * dt);
      fg[1 + _v_start + t] = v1 - (v0 + Vdot * dt);
      fg[1 + _r_start + t] = r1 - (r0 + Rdot * dt);
      // Error state
      fg[1 + _xe_start + t] = xe1 - (xe_nominal + xe_dot * dt);
      fg[1 + _ye_start + t] = ye1 - (ye_nominal + ye_dot * dt);
      fg[1 + _psie_start + t] = psie1 - (psie_nominal + psie_dot * dt);
      fg[1 + _ue_start + t] = ue1 - (ue_nominal + ue_dot * dt);
      fg[1 + _ve_start + t] = ve1 - (ve_nominal + ve_dot * dt);
      fg[1 + _re_start + t] = re1 - (ve_nominal + re_dot * dt);
    }
  }
};

//
// MPC class definition
//

Maneuver::MPC::MPC() {}
Maneuver::MPC::~MPC() {}
void Maneuver::MPC::set_params(const std::map<string, double> & params)
{
  maneuver_params = params;
}

std::vector<double> Maneuver::MPC::Solve(
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & reference_state,
  const Eigen::Vector3d & reference_acc)
{
  typedef CPPAD_TESTVECTOR (double) Dvector;
  // Initial state
  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double u = x0[3];
  double v = x0[4];
  double r = x0[5];
  double xe = x0[6];
  double ye = x0[7];
  double psie = x0[8];
  double ue = x0[9];
  double ve = x0[10];
  double re = x0[11];

  // The solver takes all the state variables and actuator
  // variables in a singular vector. Thus, we should to establish
  // when one variable starts and another ends to make our lifes easier.
  size_t N = maneuver_params["N"];
  size_t _x_start = 0;
  size_t _y_start = _x_start + N;
  size_t _psi_start = _y_start + N;
  size_t _u_start = _psi_start + N;
  size_t _v_start = _u_start + N;
  size_t _r_start = _v_start + N;
  size_t _xe_start = _r_start + N;
  size_t _ye_start = _xe_start + N;
  size_t _psie_start = _ye_start + N;
  size_t _ue_start = _psie_start + N;
  size_t _ve_start = _ue_start + N;
  size_t _re_start = _ve_start + N;
  size_t _Fx_start = _re_start + N;
  size_t _Fy_start = _Fx_start + N;
  size_t _Mz_start = _Fy_start + N - 1;

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 12 + (N - 0) * 3;
  // Number of constraints
  size_t n_constraints = N * 12 + 3 * N;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[_x_start] = x;
  vars[_y_start] = y;
  vars[_psi_start] = psi;
  vars[_u_start] = u;
  vars[_v_start] = v;
  vars[_r_start] = r;
  vars[_xe_start] = xe;
  vars[_ye_start] = ye;
  vars[_psie_start] = psie;
  vars[_ue_start] = ue;
  vars[_ve_start] = ve;
  vars[_re_start] = re;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < _psi_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_xy"];
    vars_upperbound[i] = maneuver_params["max_xy"];
  }
  for (int i = _psi_start; i < _u_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_psi"] + 0.002;
    vars_upperbound[i] = maneuver_params["max_psi"];
  }
  for (int i = _u_start; i < _v_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_u"];
    vars_upperbound[i] = maneuver_params["max_u"];
  }
  for (int i = _v_start; i < _r_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_v"];
    vars_upperbound[i] = maneuver_params["max_v"];
  }
  for (int i = _r_start; i < _xe_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_r"];
    vars_upperbound[i] = maneuver_params["max_r"];
  }
  for (int i = _xe_start; i < _psie_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_xy"];
    vars_upperbound[i] = maneuver_params["max_xy"];
  }
  for (int i = _psie_start; i < _ue_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_psi"] + 0.002;
    vars_upperbound[i] = maneuver_params["max_psi"];
  }
  for (int i = _ue_start; i < _re_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_u"];
    vars_upperbound[i] = maneuver_params["max_u"];
  }
  for (int i = _re_start; i < _Fx_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_r"];
    vars_upperbound[i] = maneuver_params["max_r"];
  }

  vars_lowerbound[_Fx_start] = prev_Fx;
  vars_upperbound[_Fx_start] = prev_Fx;

  for (int i = _Fx_start; i < _Fy_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_Fx"];
    vars_upperbound[i] = maneuver_params["max_Fx"];
  }
  vars_lowerbound[_Fy_start] = prev_Fy;
  vars_upperbound[_Fy_start] = prev_Fy;

  for (int i = _Fy_start; i < _Mz_start; i++) {
    vars_lowerbound[i] = -maneuver_params["max_Fy"];
    vars_upperbound[i] = maneuver_params["max_Fy"];
  }
  vars_lowerbound[_Mz_start] = prev_Mz;
  vars_upperbound[_Mz_start] = prev_Mz;

  for (int i = _Mz_start; i < n_vars; i++) {
    vars_lowerbound[i] = -maneuver_params["max_Mz"];
    vars_upperbound[i] = maneuver_params["max_Mz"];
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
  constraints_lowerbound[_x_start] = x;
  constraints_lowerbound[_y_start] = y;
  constraints_lowerbound[_psi_start] = psi;
  constraints_lowerbound[_u_start] = u;
  constraints_lowerbound[_v_start] = v;
  constraints_lowerbound[_r_start] = r;
  constraints_lowerbound[_xe_start] = xe;
  constraints_lowerbound[_ye_start] = ye;
  constraints_lowerbound[_psie_start] = psie;
  constraints_lowerbound[_ue_start] = ue;
  constraints_lowerbound[_ve_start] = ve;
  constraints_lowerbound[_re_start] = re;

  constraints_upperbound[_x_start] = x;
  constraints_upperbound[_y_start] = y;
  constraints_upperbound[_psi_start] = psi;
  constraints_upperbound[_u_start] = u;
  constraints_upperbound[_v_start] = v;
  constraints_upperbound[_r_start] = r;
  constraints_upperbound[_xe_start] = xe;
  constraints_upperbound[_ye_start] = ye;
  constraints_upperbound[_psie_start] = psie;
  constraints_upperbound[_ue_start] = ue;
  constraints_upperbound[_ve_start] = ve;
  constraints_upperbound[_re_start] = re;

  // Object that computes objective and constraints
  FG_eval fg_eval(reference_state, reference_acc, maneuver_params);
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
  prev_Fx = solution.x[_Fx_start + 1];
  prev_Fy = solution.x[_Fy_start + 1];
  prev_Mz = solution.x[_Mz_start + 1];

  return {solution.x[_Fx_start], solution.x[_Fy_start], solution.x[_Mz_start]};
}
