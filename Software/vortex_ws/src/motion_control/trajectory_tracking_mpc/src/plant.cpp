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

#include "plant.hpp"
// ================================================================
void Plant::initialize(
  const double & m, const double & volume,
  const Vector6d & Ib, const Vector3d & r_cob,
  const Vector3d & r_cog, const Vector6d & Ma,
  const Vector6d & Dlinear, const Vector6d & Dquad)
{
  /********************************************************************************
   * Paramteres initilization
   * r_cob_,    vector from center of Buoyancy to the origin
   * r_cog_,    vector from center of Gravity to the origin
   * Ib_,          Inertia vector [ixx, iyy, izz, ixy, ixz, iyz]
   * Mr_,         Rigid body mass matrix
   * Ma_,        Added mass matrix
   * DLinear_, Linear damping coefficients vector
   * Dquad_ ,  Quadratic damping coefficients vector
   ********************************************************************************/
  mass_ = m;
  volume_ = volume;
  r_cob_ = r_cob.cast<CppAD::AD<double>>();
  r_cog_ = r_cog.cast<CppAD::AD<double>>();
  Dlinear_ = Dlinear.cast<CppAD::AD<double>>();
  Dquad_ = Dquad.cast<CppAD::AD<double>>();
  // Rigid-Body System Inertia Matrix, (equation 3.44) Fossen 2011 book
  // Rigid-body mass matrix
  Matrix6d Mr;
  Mr.block<3, 3>(0, 0) = m * Matrix3d::Identity();
  Matrix3d inertia_matrix;
  inertia_matrix << Ib(0), Ib(3), Ib(4), Ib(3), Ib(1), Ib(5), Ib(4), Ib(5),
    Ib(2);
  Mr.setZero();
  Mr.block<3, 3>(3, 3) = inertia_matrix;
  Mr.block<3, 3>(0, 3) = -m * skew(r_cog);
  Mr.block<3, 3>(3, 0) = m * skew(r_cog);
  // Total system Mass Matrix
  Matrix6d _Ma = Ma.asDiagonal();
  M_ = Mr + _Ma;
  M_ad_ = M_.cast<CppAD::AD<double>>();
}
// ================================================================
Vector12dAD Plant::nonlinear_update(const Vector12dAD & x, const Vector6dAD & u)
{
  // Nonlinear motion model update
  // ẋ = f (x, u)
  const Vector3dAD & euler = x.segment<3>(3);
  const Vector6dAD & nu = x.tail<6>();
  Vector12dAD dx;
  dx.segment<6>(0) = nonlinear_kinematics(euler, nu);
  Vector6dAD fd = nonlinear_damping_forces(nu);
  Vector6dAD fc = nonlinear_coriolis_forces(nu);
  Vector6dAD fg = nonlinear_restoring_forces(euler);
  dx.segment<6>(6) = M_ad_.inverse() * (u - fd - fc - fg);
  return dx;
}
// ================================================================
Vector6dAD Plant::nonlinear_kinematics(
  const Vector3dAD & euler,
  const Vector6dAD & nu)
{
  /* kinematics transformation matrix (J) 6x6 matrix for transforming  6DOF
   * velocity from Body to Inertial frame Reference: equation (2.40) Fossen 2011
   */
  Matrix6dAD J = Matrix6dAD::Zero();
  J.block<3, 3>(0, 0) = RBtoI(euler);
  J.block<3, 3>(3, 3) = TBtoI(euler);
  Vector6dAD eta_dot = J * nu;
  return eta_dot;
}
// ====================================================================
Vector6d Plant::kinematics_inverse(
  const Vector3d & euler,
  const Vector6d & eta_dot)
{
  /* kinematics inverse transformation matrix (J ^ -1) 6x6 matrix for transforming  6DOF
   * velocity from Inertial to Body frame Reference: equation (2.24, 2.26) Fossen 2011
   */
  Matrix6dAD J_inv = Matrix6dAD::Zero();
  J_inv.block<3, 3>(0, 0) = RBtoI(euler.cast<CppAD::AD<double>>()).transpose();
  J_inv.block<3, 3>(3, 3) = TItoB(euler.cast<CppAD::AD<double>>());
  Vector6dAD nu_ad = J_inv * eta_dot.cast<CppAD::AD<double>>();
  using CppAD::Value;
  Vector6d nu;
  for (size_t i = 0; i < 6; i++) {
    nu(i) = Value(nu_ad(i));
  }
  return nu;
}
// ================================================================
Vector6dAD Plant::nonlinear_damping_forces(const Vector6dAD & nu)
{
  /* Calculating the total damping matrix D
   * D(v) = D_linear + D_nonlinear * |nu|)
   * [nu] is the body velocity vector
   */
  Matrix6dAD D = -1 * Dlinear_.asDiagonal();
  D += -1 * (Dquad_.cwiseProduct(nu.cwiseAbs())).asDiagonal();
  Vector6dAD damping_forces = D * nu;
  return damping_forces;
}
// ================================================================
Vector6dAD Plant::nonlinear_coriolis_forces(const Vector6dAD & nu)
{
  /* Coriolis–Centripetal forces fc.
   * [nu] is the body velocity vector
   * fc = C(nu) * nu
   * C(nu) is the  Coriolis–Centripetal Matrix calculated from the System mass
   * matrix M Reference: equation (3.46) Fossen 2011
   */
  Matrix6dAD C = Matrix6dAD::Zero();
  C.block<3, 3>(0, 3) = -skew_ad(
    M_ad_.block<3, 3>(0, 0) * nu.head<3>() +
    M_ad_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 0) = -skew_ad(
    M_ad_.block<3, 3>(0, 0) * nu.head<3>() +
    M_ad_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 3) = -skew_ad(
    M_ad_.block<3, 3>(3, 0) * nu.head<3>() +
    M_ad_.block<3, 3>(3, 3) * nu.tail<3>());
  Vector6dAD coriolis_forces = C * nu;
  return coriolis_forces;
}
// ================================================================
Vector6dAD Plant::nonlinear_restoring_forces(const Vector3dAD & euler)
{
  /* Restoring forces and moments
   * Reference: equation (4.5) Fossen 2011
   */
  // Gravity force
  Vector3dAD Fg{0.0, 0.0, mass_ * 9.806};
  // Buoyancy force
  Vector3dAD Fb{0.0, 0.0, -volume_ * 9.806 * 1028};
  // Rotation from  Inertial to body
  Matrix3dAD R = RBtoI(euler).transpose();
  Vector6dAD g = Vector6dAD::Zero();
  g << -(R * (Fg + Fb)),
    -((skew_ad(r_cog_) * R * Fg + skew_ad(r_cob_) * R * Fb));
  return g;
}
// ================================================================
Matrix3dAD Plant::RBtoI(const Vector3dAD & euler)
{
  /* Rotation matrix from Body frame to Inertial frame
   * Reference: equation (2.18) Fossen 2011
   * euler -> rpy w.r.t  inertial NED
   */
  CppAD::AD<double> cphi = CppAD::cos(euler(0));
  CppAD::AD<double> sphi = CppAD::sin(euler(0));
  CppAD::AD<double> cth = CppAD::cos(euler(1));
  CppAD::AD<double> sth = CppAD::sin(euler(1));
  CppAD::AD<double> cpsi = CppAD::cos(euler(2));
  CppAD::AD<double> spsi = CppAD::sin(euler(2));
  Matrix3dAD R;
  R << cpsi * cth, -spsi * cphi + cpsi * sth * sphi,
    spsi * sphi + cpsi * cphi * sth, spsi * cth,
    cpsi * cphi + sphi * sth * spsi, -cpsi * sphi + sth * spsi * cphi, -sth,
    cth * sphi, cth * cphi;
  return R;
}
// ================================================================
Matrix3dAD Plant::TBtoI(const Vector3dAD & euler)
{
  /* Transformation matrix from Body frame to Inertial frame
   * Reference: equation (2.28.b) Fossen 2011
   */
  CppAD::AD<double> cphi = CppAD::cos(euler(0));
  CppAD::AD<double> sphi = CppAD::sin(euler(0));
  CppAD::AD<double> cth = CppAD::cos(euler(1));
  // cth = CppAD::sign(cth) * CppAD::min(0.01745, CppAD::abs(cth));
  CppAD::AD<double> sth = CppAD::sin(euler(1));
  Matrix3dAD T;
  T << 1.0, sphi * sth / cth, cphi * sth / cth, 0.0, cphi, -sphi, 0.0,
    sphi / cth, cphi / cth;
  return T;
}
// ================================================================
Matrix3dAD Plant::TItoB(const Vector3dAD & euler)
{
  /* Transformation matrix from Inertial frame to body frame
   * Reference: equation (2.28.a) Fossen 2011
   */
  CppAD::AD<double> cphi = CppAD::cos(euler(0));
  CppAD::AD<double> sphi = CppAD::sin(euler(0));
  CppAD::AD<double> cth = CppAD::cos(euler(1));
  CppAD::AD<double> sth = CppAD::sin(euler(1));
  Matrix3dAD T;
  T << 1.0, -sth, 0.0, 0.0, cphi, cth * sphi, 0.0, -sphi, cth * cphi;
  return T;
}
// ================================================================
Matrix3dAD Plant::skew_ad(const Vector3dAD & v)
{
  // Skew symmetric matrix
  // Matrix for cross-product operation of a vector
  Matrix3dAD S;
  S << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return S;
}
Matrix3d Plant::skew(const Vector3d & v)
{
  Matrix3d S;
  S << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return S;
}
