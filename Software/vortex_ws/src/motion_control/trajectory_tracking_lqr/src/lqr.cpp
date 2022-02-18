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

void LQR::set_params(
  const double & m, const double & volume, const Vector6d & Ib,
  const Vector3d & r_cob, const Vector3d & r_cog,
  const Vector6d & Ma, const Vector6d & Dlinear,
  const Vector6d & Dquad, const Vector12d & Q,
  const Vector6d & R, const Vector6d & tau_max,
  const Vector12d & error_max)
{
  /* Paramteres initilization for SWIFT AUV
   * r_cob_ -> vector from center of Buoyancy to the origin
   * r_cog_ -> vector from center of Gravity to the origin
   * Ib_ -> Inertia vector [ixx, iyy, izz, ixy, ixz, iyz]
   * Mr_ -> Rigid body mass matrix
   * Ma_ -> Added mass matrix
   * DLinear_ -> Linear damping coefficients vector
   * Dquad_ -> Quadratic damping coefficients vector
   * Q_ -> the state weighting matrix for the optimization problem
   * R_ -> the control forces weighting matrix the optimization problem
   */
  this->mass_ = m;
  this->volume_ = volume;
  this->Ib_ = Ib;
  this->r_cob_ = r_cob;
  this->r_cog_ = r_cog;
  this->Ma_ = Ma.asDiagonal();
  this->Dlinear_ = Dlinear.asDiagonal();
  this->Dquad_ = Dquad.asDiagonal();
  this->Q_ = Q.asDiagonal();
  this->R_ = R.asDiagonal();
  this->tau_max_ = tau_max;
  this->error_max_ = error_max;
  // Rigid-Body System Inertia Matrix, (equation 3.44) Fossen 2011 book
  this->Mr_.block<3, 3>(0, 0) = mass_ * Matrix3d::Identity();
  Matrix3d inertia_matrix;
  inertia_matrix << Ib_(0), Ib_(3), Ib_(4), Ib_(3), Ib_(1), Ib_(5), Ib_(4),
    Ib_(5), Ib_(2);
  this->Mr_.block<3, 3>(3, 3) = inertia_matrix;
  this->Mr_.block<3, 3>(0, 3) = -mass_ * skew(r_cog_);
  this->Mr_.block<3, 3>(3, 0) = mass_ * skew(r_cog_);
  // Total Mass Matrix
  this->M_ = this->Mr_ + this->Ma_;
}

Matrix6d LQR::calculate_damping_matrix(Vector6d & nu)
{
  /* Calculating the total damping matrix D
   * D(v) = D_linear + D_nonlinear * |nu|)
   * [nu] is the body velocity vector
   */
  Matrix6d D = Dlinear_;
  D(0, 0) += Dquad_(0, 0) * std::abs(nu(0));
  D(1, 1) += Dquad_(1, 1) * std::abs(nu(1));
  D(2, 2) += Dquad_(2, 2) * std::abs(nu(2));
  D(3, 3) += Dquad_(3, 3) * std::abs(nu(3));
  D(4, 4) += Dquad_(4, 4) * std::abs(nu(4));
  D(5, 5) += Dquad_(5, 5) * std::abs(nu(5));
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
   * Reference: equation (4.5) Fossen 2011
   */
  // Gravity froce
  Vector3d Fg{0.0, 0.0, mass_ * 9.806};
  // Buoyancy force
  Vector3d Fb{0.0, 0.0, -volume_ * 1000 * 9.896};
  // rotation from Body to Inertial
  Matrix3d R = RBtoI(euler);
  Vector6d g;
  g << -(R * (Fg + Fb)), -(R * (skew(r_cog_) * Fg + skew(r_cob_) * Fb));
  return g;
}
void LQR::saturate_control(Vector6d & tau)
{
  /* Saturating the control forces and moments at ± 40 [N, N.m]
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
  /* Saturating the state errors ± 0.8 for avoiding agressive control actions
   */
  for (int i = 0; i <= 11; i++) {
    if (delta(i) > error_max_(i)) {
      delta(i) = error_max_(i);
    } else if (delta(i) < -error_max_(i)) {
      delta(i) = -error_max_(i);
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
  double cphi = cos(euler(0));
  double sphi = sin(euler(0));
  double cth = cos(euler(1));
  double sth = sin(euler(1));
  double cpsi = cos(euler(2));
  double spsi = sin(euler(2));
  Matrix3d R;
  R << cpsi * cth, -spsi * cphi + cpsi * sth * sphi,
    spsi * sphi + cpsi * cphi * sth, spsi * cth,
    cpsi * cphi + sphi * sth * spsi, -cpsi * sphi + sth * spsi * cphi, -sth,
    cth * sphi, cth * cphi;
  return R;
}
Matrix3d LQR::TBtoI(Vector3d & euler)
{
  /* Translation matrix from Body frame to Inertial frame
   * Reference: equation (2.28.b) Fossen 2011
   */
  double cphi = cos(euler(0));
  double sphi = sin(euler(0));
  double cth = cos(euler(1));
  cth = sign(cth) * std::min(0.01745, abs(cth));
  double sth = sin(euler(1));
  Matrix3d T;
  T <<
    1.0, sphi * sth / cth, cphi * sth / cth,
    0.0, cphi, -sphi,
    0.0, sphi / cth, cphi / cth;
  return T;
}
Matrix3d LQR::TItoB(Vector3d & euler)
{
  /* Translation matrix from Inertial frame to body frame
   * Reference: equation (2.28.a) Fossen 2011
   */
  double cphi = cos(euler(0));
  double sphi = sin(euler(0));
  double cth = cos(euler(1));
  double sth = sin(euler(1));
  Matrix3d T;
  T <<
    1.0, -sth, 0.0,
    0.0, cphi, cth * sphi,
    0.0, -sphi, cth * cphi;
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
