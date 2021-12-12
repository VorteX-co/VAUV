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
   * rb_ -> vector from center of Buoyancy to the origin
   * rg_ -> vector from center of Gravity to the origin
   * Ib_ -> Inertia matrix
   * Mr_ -> Rigid body mass matrix
   * Ma_ -> Added mass matrix
   * DL_ -> Linear damping coefficients vector
   * DNL_ -> Quadratic damping coefficients vector
   * Q_ -> the state weighting matrix for the optimization problem
   * R_ -> the control forces weighting matrix the optimization problem
   */
  // Center of Gravitiy CG is the origin
  rg_(0, 0) = 0.0;
  rg_(1, 0) = 0.0;
  rg_(2, 0) = 0.0;
  rb_(0, 0) = 0.0;
  rb_(1, 0) = 0.0;
  // All the equations are derived in NED frame, CB is above the CG therefore it
  // has a -ve sign in z
  rb_(2, 0) = -0.12489;
  mass_ = 35.5;
  volume_ = 0.0364;
  // Inertia matrix
  Ib_(0, 0) = 0.8061;
  Ib_(0, 1) = -0.0059;
  Ib_(0, 2) = 0.0005;
  Ib_(1, 0) = -0.0059;
  Ib_(1, 1) = 0.8;
  Ib_(1, 2) = -0.0113;
  Ib_(2, 0) = 0.0005;
  Ib_(2, 1) = -0.0113;
  Ib_(2, 2) = 1.5599;
  // Rigid-Body System Inertia Matrix, (equation 3.44) Fossen 2011 book
  Mr_.block<3, 3>(0, 0) = mass_ * Matrix3d::Identity();
  Mr_.block<3, 3>(3, 3) = Ib_;
  Mr_.block<3, 3>(0, 3) = -mass_ * skew(rg_);
  Mr_.block<3, 3>(3, 0) = mass_ * skew(rg_);
  Ma_(0, 0) = 13;
  Ma_(1, 1) = 20;
  Ma_(2, 2) = 35;
  Ma_(3, 3) = 12.2;
  Ma_(4, 4) = 12.37;
  Ma_(5, 5) = 7.3;
  M_ = Mr_ + Ma_;
  DL_(0, 0) = -22.8;
  DL_(1, 1) = -30.95;
  DL_(2, 2) = -50.26;
  DL_(3, 3) = -16.05;
  DL_(4, 4) = -16.73;
  DL_(5, 5) = -5.13;
  DNL_(0, 0) = -28.43;
  DNL_(1, 1) = -55.98;
  DNL_(2, 2) = -137.5;
  DNL_(3, 3) = 0.0;
  DNL_(4, 4) = 0.0;
  DNL_(5, 5) = 0.0;
  Q_ = Matrix12d::Identity();
  Q_(0, 0) = 2.5;
  Q_(1, 1) = 2.5;
  Q_(2, 2) = 2.5;
  R_ = 0.01 * Matrix6d::Identity();
}
Matrix6d LQR::calculate_damping_matrix(Vector6d & nu)
{
  /* Calculating the total damping matrix D
   * D(v) = D_linear + D_nonlinear * |nu|)
   * [nu] is the body velocity vector
   */
  Matrix6d D = DL_;
  D(0, 0) += DNL_(0, 0) * std::abs(nu(0));
  D(1, 1) += DNL_(1, 1) * std::abs(nu(1));
  D(2, 2) += DNL_(2, 2) * std::abs(nu(2));
  D(3, 3) += DNL_(3, 3) * std::abs(nu(3));
  D(4, 4) += DNL_(4, 4) * std::abs(nu(4));
  D(5, 5) += DNL_(5, 5) * std::abs(nu(5));
  return D;
}
Matrix6d LQR::calculate_jacobian_matrix(Vector3d & euler)
{
  /* Jacobian 6x6 matrix for transformation of 6DOF velocity from Body to
   * Inertial frame Reference: equation (2.40) Fossen 2011
   */
  Matrix6d J = Matrix6d::Zero();
  J.block<3, 3>(0, 0) = RBtoI(euler);
  J.block<3, 3>(3, 3) = TBtoI(euler);
  return J;
}
Matrix6d LQR::calculate_coriolis_matrix(Vector6d & nu)
{
  /* Coriolis–Centripetal Matrix from System Inertia Matrix M
   * Reference: equation (3.46) Fossen 2011
   * skew is the cross-product operator provided
   * [nu] is the body velocity vector
   */
  Matrix6d C = Matrix6d::Zero();
  C.block<3, 3>(0, 3) = -skew(M_.block<3, 3>(0, 0) * nu.head<3>() +
      M_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 0) = -skew(M_.block<3, 3>(0, 0) * nu.head<3>() +
      M_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 3) = -skew(M_.block<3, 3>(3, 0) * nu.head<3>() +
      M_.block<3, 3>(3, 3) * nu.tail<3>());
  return C;
}
Vector6d LQR::calculate_restoring_forces(Vector3d & euler)
{
  /* Restoring forces and moments
   * Reference: equation (4.6) Fossen 2011
   */
  double W = mass_ * 9.81;
  double B = volume_ * 1028 * 9.81;
  Vector6d g;
  g(0, 0) = (W - B) * sin(euler(1));
  g(1, 0) = -(W - B) * cos(euler(1)) * sin(euler(0));
  g(2, 0) = -(W - B) * cos(euler(1)) * cos(euler(0));
  g(3, 0) = -(rg_(1, 0) * W - rb_(1, 0) * B) * cos(euler(1)) * cos(euler(0)) +
    (rg_(2, 0) * W - rb_(2, 0) * B) * cos(euler(1)) * sin(euler(0));
  g(4, 0) = (rg_(2, 0) * W - rb_(2, 0) * B) * sin(euler(1)) +
    (rg_(0, 0) * W - rb_(0, 0) * B) * cos(euler(1)) * cos(euler(0));
  g(5, 0) = -(rg_(0, 0) * W - rb_(0, 0) * B) * cos(euler(1)) * sin(euler(0)) -
    (rg_(1, 0) * W - rb_(1, 0) * B) * sin(euler(1));
  return g;
}
void LQR::saturate_control(Vector6d & tau)
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
}
void LQR::saturate_error(Vector12d & delta)
{
  /* Saturating the state errors ± 0.8 for avoiding agressive control actions
   */
  for (int i = 0; i <= 11; i++) {
    if (delta(i) > 0.8) {
      delta(i) = 0.8;
    } else if (delta(i) < -0.8) {
      delta(i) = -0.8;
    }
  }
}
void LQR::to_SNAME(Vector12d & x)
{
  /* from ENU to NED, since all the equations in the LQR class are derived in
   * inertial NED frame.
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
Matrix3d LQR::RBtoI(Vector3d & euler)
{
  /* Rotation matrix from Body frame to Inertial frame
   * Reference: equation (2.18) Fossen 2011
   * euler -> rpy w.r.t  inertial NED
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
Matrix3d LQR::TBtoI(Vector3d & euler)
{
  /* Translation matrix from Body frame to Inertial frame
   * Reference: equation (2.28.b) Fossen 2011
   */
  double cp = cos(euler(1));
  cp = sign(cp) * std::min(0.01745, abs(cp));
  Matrix3d T;
  T << 1 * cp, sin(euler(0)) * sin(euler(1)), cos(euler(0)) * sin(euler(1)), 0,
    cos(euler(0)) * cp, -cp * sin(euler(0)), 0, sin(euler(0)), cos(euler(0));
  T *= 1 / cp;
  return T;
}
Matrix3d LQR::TItoB(Vector3d & euler)
{
  /* Translation matrix from Inertial frame to body frame
   * Reference: equation (2.28.a) Fossen 2011
   */
  Matrix3d T;
  T << 1, 0, -sin(euler(1)), 0, cos(euler(0)), cos(euler(1)) * sin(euler(0)), 0,
    -sin(euler(0)), cos(euler(1)) * cos(euler(0));
  return T;
}
Vector6d LQR::action(Vector12d & x, Vector12d & xd, Vector6d & ffacc)
{
  /* Calculating LQR Control action
   * The state vector and desired state vector are in ENU frame
   *
   */
  to_SNAME(x);
  to_SNAME(xd);
  /* Constructing the linear paramter varying (LPV) state space model
   *
   * The state vector x [η,ν]
   * The input vector  τ [f,m]
   *
   * Nonlinear 6 DOF equations of motion  equations (7.185, 7.186) Fossen 2011:
   *
   *                          η̇ = J (η)∗ ν
   *  M ∗ ν̇ + C ∗ ν + D ∗ ν + g(η)  = τ
   *
   * In state  space form ẋ = Ax + Bu, Augmenting the tracking error ė = Ae + Bu
   * The rule of the LQR is to regulate the tracking error e via u = τ_lqr
   *
   *  A =    |    0(6x6)     J(η)                         |
   *            |    0(6X6)   -(M^−1)[C(ν)+D(ν)]  |
   *
   * B =     |   0(6x6)     |
   *            |   (M^−1)    |
   *
   * The restoring forces g(η) are compensated by adding a feedforward control
   * law
   *
   * τ = g(η) + τ_lqr
   */
  // Attitude
  Vector3d euler = x.block<3, 1>(3, 0);
  Matrix6d J = calculate_jacobian_matrix(euler);
  // Velocity in body frame
  Vector6d nu = x.tail<6>();
  Matrix6d D = calculate_damping_matrix(nu);
  Matrix6d C = calculate_coriolis_matrix(nu);
  Matrix12d A = Matrix12d::Zero();
  A.block<6, 6>(0, 6) = J;
  A.block<6, 6>(6, 6) = -M_.inverse() * (D + C);
  Eigen::Matrix<double, 12, 6> B = Eigen::Matrix<double, 12, 6>::Zero();
  B.block<6, 6>(6, 0) = M_.inverse();
  // Solving the algebraic Riccati equation (ARE)
  Matrix12d P = Matrix12d::Zero();
  care_solver_.solve(P, A, B, Q_, R_);
  // Compute the optimal feedback gain matrix K, reference equation (13.8)
  // Fossen 2011
  Eigen::Matrix<double, 6, 12> K = -R_.inverse() * B.transpose() * P;
  // Compute The optimal feedback control forces
  Vector12d e = x - xd;
  saturate_error(e);
  Vector6d tau_lqr = K * e;
  // For linearization we put resetoring forces g(η) as feedforward control
  Vector6d tau_ff = calculate_restoring_forces(euler);
  // The total control signal
  Vector6d u = tau_ff + tau_lqr;
  saturate_control(u);
  return u;
}
