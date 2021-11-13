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
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include "geometry/support.h"

LQR::LQR()
{
  /* Paramteres initilization for SWIFT AUV
   * rb -> vector from center of volume to the origin
   * rg -> vector from center of gravity to the origin
   * Ib -> Inertia matrix
   * Mr -> Rigid body mass matrix
   * Ma -> Added mass matrix
   * DL -> Linear damping coefficients vector
   * DNL -> Quadratic damping coefficients vector
   * Q -> the state weighting matrix for the optimization problem
   * R -> the control forces weighting matrix the optimization problem
   */
  rb(0, 0) = 0.0;
  rb(1, 0) = 0.0;
  rb(2, 0) = -0.12489;
  rg(0, 0) = 0.0;
  rg(1, 0) = 0.0;
  rg(2, 0) = 0.0;
  mass = 35.5;
  volume = 0.0364;
  Ib(0, 0) = 0.8061;
  Ib(0, 1) = -0.0059;
  Ib(0, 2) = 0.0005;
  Ib(1, 0) = -0.0059;
  Ib(1, 1) = 0.8;
  Ib(1, 2) = -0.0113;
  Ib(2, 0) = 0.0005;
  Ib(2, 1) = -0.0113;
  Ib(2, 2) = 1.5599;
  Mr.block<3, 3>(0, 0) = mass * Matrix3d::Identity();
  Mr.block<3, 3>(3, 3) = Ib;
  Mr.block<3, 3>(0, 3) = -mass * skew(rg);
  Mr.block<3, 3>(3, 0) = mass * skew(rg);
  Ma(0, 0) = 13;
  Ma(1, 1) = 20;
  Ma(2, 2) = 35;
  Ma(3, 3) = 12.2;
  Ma(4, 4) = 12.37;
  Ma(5, 5) = 7.3;
  M = Mr + Ma;
  DL(0, 0) = -22.8;
  DL(1, 1) = -30.95;
  DL(2, 2) = -50.26;
  DL(3, 3) = -16.05;
  DL(4, 4) = -16.73;
  DL(5, 5) = -5.13;
  DNL(0, 0) = -28.43;
  DNL(1, 1) = -55.98;
  DNL(2, 2) = -137.5;
  DNL(3, 3) = 0.0;
  DNL(4, 4) = 0.0;
  DNL(5, 5) = 0.0;
  Q = Matrix12d::Identity();
  Q(0, 0) = 5;
  Q(1, 1) = 5;
  Q(2, 2) = 5;
  Q(5, 5) = 5;
  R = 0.01 * Matrix6d::Identity();
}
Matrix6d LQR::calculate_damping_matrix()
{
  /* Calculating the total damping matrix D
   */
  Eigen::Matrix<double, 6, 6> D = -DL;
  D(0, 0) += -DNL(0, 0) * std::abs(nu(0));
  D(1, 1) += -DNL(1, 1) * std::abs(nu(1));
  D(2, 2) += -DNL(2, 2) * std::abs(nu(2));
  D(3, 3) += -DNL(3, 3) * std::abs(nu(3));
  D(4, 4) += -DNL(4, 4) * std::abs(nu(4));
  D(5, 5) += -DNL(5, 5) * std::abs(nu(5));
  return D;
}
Matrix6d LQR::calculate_jacobian_matrix()
{
  /* Jacobian 6x6 matrix for transformation of 6DOF velocity from Body to
   * Inertial frame Reference: equation (2.40) Fossen 2011 J(η) =    |  R 0(3x3)
   * | |  0(3x3)   T          |
   *
   */
  Matrix6d J = Matrix6d::Zero();
  J.block<3, 3>(0, 0) = RBtoI(eta.tail<3>());
  J.block<3, 3>(3, 3) = TBtoI(eta.tail<3>());
  return J;
}
Matrix6d LQR::calculate_coriolis_matrix()
{
  /* Coriolis–Centripetal Matrix from System Inertia Matrix M
   *  Reference: equation (3.46) Fossen 2011
   *  skew is a function from geometry library for the cross-product operator
   *
   * C(ν) =  |    0(3×3)                          −S(M11 ν1 + M12 ν2 )      |
   *             |  −S(M11 ν1 + M12 ν2)    −S(M 21 ν 1 + M22 ν2 )    |
   *
   * ν1 := [u, v, w]
   * ν2 := [p, q, r]
   *
   */
  Matrix6d C = Matrix6d::Zero();
  C.block<3, 3>(0, 3) = -skew(M.block<3, 3>(0, 0) * nu.head<3>() +
      M.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 0) = -skew(M.block<3, 3>(0, 0) * nu.head<3>() +
      M.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 3) = -skew(M.block<3, 3>(3, 0) * nu.head<3>() +
      M.block<3, 3>(3, 3) * nu.tail<3>());
  return C;
}
Vector6d LQR::calculate_restoring_forces()
{
  /* Restoring forces and moments in NED frame
   * Reference: equation (4.6) Fossen 2011
   */
  double W = mass * 9.81;
  double B = volume * 1028 * 9.81;
  Vector6d g;
  g(0, 0) = (W - B) * sin(eta(4));
  g(1, 0) = -(W - B) * cos(eta(4)) * sin(eta(3));
  g(2, 0) = -(W - B) * cos(eta(4)) * cos(eta(3));
  g(3, 0) = -(rg(1, 0) * W - rb(1, 0) * B) * cos(eta(4)) * cos(eta(3)) +
    (rg(2, 0) * W - rb(2, 0) * B) * cos(eta(4)) * sin(eta(3));
  g(4, 0) = (rg(2, 0) * W - rb(2, 0) * B) * sin(eta(4)) +
    (rg(0, 0) * W - rb(0, 0) * B) * cos(eta(4)) * cos(eta(3));
  g(5, 0) = -(rg(0, 0) * W - rb(0, 0) * B) * cos(eta(4)) * sin(eta(3)) -
    (rg(1, 0) * W - rb(1, 0) * B) * sin(eta(4));
  return g;
}
Vector6d LQR::saturate_control(Vector6d tau)
{
  /* Saturating the control forces and moments at ± 40 [N, N.m]
   */
  for (int i = 0; i <= 5; i++) {
    if (tau(i) > 40) {
      tau(i) = 40;
    } else if (tau(i) < -40) {
      tau(i) = -40;
    }
  }
  return tau;
}
Vector12d LQR::saturate_error(Vector12d delta)
{
  /* Saturating the state errors ± 0.8 for avoiding agressive control actions
   */
  for (int i = 0; i <= 5; i++) {
    if (delta(i) > 0.8) {
      delta(i) = 0.8;
    } else if (delta(i) < -0.8) {
      delta(i) = -0.8;
    }
  }
  return delta;
}
void LQR::to_SNAME(Vector6d & x)
{
  /* from ENU to NED for the calculation in LQR class
   */
  x(1) *= -1;
  x(2) *= -1;
  x(4) *= -1;
  x(5) *= -1;
}
Matrix3d LQR::RBtoI(Vector3d euler)
{
  /* Rotation matrix from Body frame to Inertial frame
   * Reference: equation (2.18) Fossen 2011
   */
  Matrix3d R;
  R(0, 0) = cos(euler(2)) * cos(euler(1));
  R(0, 1) = -sin(euler(2)) * cos(euler(0)) +
    cos(euler(2)) * sin(euler(1)) * sin(euler(0));
  R(0, 2) = sin(euler(2)) * sin(euler(0)) +
    cos(euler(2)) * cos(euler(0)) * sin(euler(1));
  R(1, 0) = sin(euler(2)) * cos(euler(1));
  R(1, 1) = cos(euler(2)) * cos(euler(0)) +
    sin(euler(0)) * sin(euler(2)) * sin(euler(1));
  R(1, 2) = -cos(euler(2)) * sin(euler(0)) +
    sin(euler(1)) * sin(euler(2)) * cos(euler(0));
  R(2, 0) = -sin(euler(1));
  R(2, 1) = cos(euler(1)) * sin(euler(0));
  R(2, 2) = cos(euler(1)) * cos(euler(0));
  return R;
}
Matrix3d LQR::TBtoI(Vector3d euler)
{
  /* Translation matrix from Body frame to Inertial frame
   * Reference: equation (2.28.b) Fossen 2011
   */
  double cp = cos(euler(1));
  cp = sign(cp) * min(0.01745, abs(cp));
  Matrix3d T;
  T << 1 * cp, sin(euler(0)) * sin(euler(1)), cos(euler(0)) * sin(euler(1)), 0,
    cos(euler(0)) * cp, -cp * sin(euler(0)), 0, sin(euler(0)), cos(euler(0));
  T *= 1 / cp;
  return T;
}
Matrix3d LQR::TItoB(Vector3d euler)
{
  /* Translation matrix from Inertial frame to body frame
   * Reference: equation (2.28.a) Fossen 2011
   */
  Matrix3d T;
  T << 1, 0, -sin(euler(1)), 0, cos(euler(0)), cos(euler(1)) * sin(euler(0)), 0,
    -sin(euler(0)), cos(euler(1)) * cos(euler(0));
  return T;
}
Vector6d LQR::action(Vector12d x, Vector12d xd, Vector6d ffacc)
{
  /* Calculating LQR Control action
   * The state vector and desired state vector are in ENU frame
   * The function to_SNAME(Vector6d) converts ENU <> NED
   */
  eta = x.head<6>();
  to_SNAME(eta);
  x.block<6, 1>(0, 0) = eta;
  nu = x.tail<6>();
  to_SNAME(nu);
  x.block<6, 1>(6, 0) = nu;
  Vector6d eta_des = xd.head<6>();
  to_SNAME(eta_des);
  xd.block<6, 1>(0, 0) = eta_des;
  Vector6d nu_des = xd.tail<6>();
  to_SNAME(nu_des);
  xd.block<6, 1>(6, 0) = nu_des;
  //
  /* Constructing the linear paramter varying (LPV) state space model
   *
   * The state vector x [η,ν]
   * The input vector  τ [f,m]
   *
   * Nonlinear 6 DOF equations of motion  equations (7.185, 7.186) Fossen 2011:
   *
   *                          η̇ = J (η)∗ ν
   *  M ∗ ν̇ + C ∗ ν +   + D ∗ ν + g(η)  = τ
   *
   * In state  space form ẋ = Ax + Bu  , Augmenting the tracking error ė = Ae +
   * Bu
   *
   * | η̇ - η̇d |      |          0(6x6)         J(η) |       | η - ηd | | ν̇ - ν̇d
   * |   = |          0(6x6)       −(M^−1) ∗ [C(ν)ν + D(ν)ν]   |  ∗  | ν - νd  |
   * +
   *
   *               |                             0(6x6) |       ∗    τ_lqr |
   * (M^−1)                               |
   *
   *  A =    |    0(6x6)     J(η)                         |
   *            |    0(6X6)   -(M^−1)[C(ν)+D(ν)]  |
   *
   * B =     |   0(6x6)     |
   *            |   (M^−1)    |
   *
   * The restoring forces g(η) are compensated by adding a feedforward control
   * low
   *
   * τ = g(η) + τ_lqr
   */
  Matrix12d A = Matrix12d::Zero();
  Matrix6d D = calculate_damping_matrix();
  Matrix6d J = calculate_jacobian_matrix();
  Matrix6d C = calculate_coriolis_matrix();

  A.block<6, 6>(0, 6) = J;
  A.block<6, 6>(6, 6) = -M.inverse() * (D + C);
  Eigen::Matrix<double, 12, 6> B;
  B.setZero();
  B.block<6, 6>(6, 0) = M.inverse();
  // Solving the algebraic Riccati equation (ARE)
  Matrix12d P;
  P.setZero();
  care_solver.solve(P, A, B, Q, R);
  // Compute the optimal feedback gain matrix K, reference equation (13.8)
  // Fossen 2011
  Eigen::Matrix<double, 6, 12> K = -R.inverse() * B.transpose() * P;
  // Compute The optimal feedback control forces
  Vector6d tau_lqr = K * saturate_error(x - xd);
  // For linearization we put resetoring forces g(η) as feedforward control
  // input
  Vector6d tau_ff = calculate_restoring_forces();
  // Logging the control state
  Eigen::Matrix<double, 12, 3> logMatrix;
  logMatrix << x, xd, x - xd;
  std::cout << " ******** Logging NED frame info *********" << std::endl;
  std::cout << " ****** x ********** xd ********** error *****" << std::endl;
  std::cout << logMatrix << std::endl;
  return saturate_control(tau_lqr + tau_ff);
}
