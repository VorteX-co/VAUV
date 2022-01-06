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

#include "plant.hpp"
#include "eigen_headers.hpp"
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
Vector12dAD Plant::nonlinear_update(const Vector12dAD & x, const Vector6dAD & u)
{
  // ẋ = f (x, u)
  const Vector6dAD & nu = x.tail<6>();
  const Vector3dAD & euler = x.segment<3>(3);
  Vector12dAD dx;
  dx.segment<6>(0) = nonlinear_kinematics(euler, nu);
  Vector6dAD fd = nonlinear_damping_forces(nu);
  Vector6dAD fc = nonlinear_coriolis_forces(nu);
  Vector6dAD fg = nonlinear_restoring_forces(euler);
  dx.segment<6>(6) = M_ad_.inverse() * (u - fd - fc - fg);
  return dx;
}
// ================================================================
void Plant::linearize(
  const Vector12d & x, const Vector6d & u, Matrix12d & A,
  Eigen::Matrix<double, 12, 6> & B)
{
  // See, H2 and H∞ Designs for Diving and Course Control of an Autonomous
  // Underwater Vehicle in Presence of Waves, Lúcia Moreira. Equation (15) for
  // the notations.
  Matrix6d J, J_star, C, D, G;
  kinematics_jacobian(x.segment<6>(0), x.segment<6>(6), J, J_star);
  G = restoring_forces_jacobian(x.segment<6>(0));
  D = damping_forces_jacobian(x.segment<6>(6));
  C = coriolis_forces_jacobian(x.segment<6>(6));
  A.block<6, 6>(0, 0) = J_star;
  A.block<6, 6>(0, 6) = J;
  A.block<6, 6>(6, 0) = -M_.inverse() * G;
  A.block<6, 6>(6, 6) = -M_.inverse() * (C + D);
  B.setZero();
  B.block<6, 6>(6, 0) = M_.inverse();
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
  C.block<3, 3>(0, 3) = -skew_ad(M_ad_.block<3, 3>(0, 0) * nu.head<3>() +
      M_ad_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 0) = -skew_ad(M_ad_.block<3, 3>(0, 0) * nu.head<3>() +
      M_ad_.block<3, 3>(0, 3) * nu.tail<3>());
  C.block<3, 3>(3, 3) = -skew_ad(M_ad_.block<3, 3>(3, 0) * nu.head<3>() +
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
  // Gravity froce
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
Matrix6d Plant::damping_forces_jacobian(const Vector6d & nu)
{
  // Dynamic vector based on CppAD::AD<double>
  VectorXdAD _nu(6);
  _nu = nu.cast<CppAD::AD<double>>();
  // Declare independent variables and starting recording
  CppAD::Independent(_nu);
  // Nonlinear damping forces
  VectorXdAD fd = nonlinear_damping_forces(_nu);
  // Create f: _nu -> fd and stop tape recording
  auto f = CppAD::ADFun<double>(_nu, fd);
  // Equilibrium state / operating point for linearization
  VectorXd nu_eq(6);
  nu_eq.setZero();
  nu_eq.segment<3>(0) = nu.head<3>() * 0.5;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> D;
  Eigen::Map<VectorXd> J_map(D.data(), D.size());
  // Compute the derivative fd'(nu_eq)
  J_map << f.Jacobian(nu_eq);
  return D;
}

// =========================================================================
void Plant::kinematics_jacobian(
  const Vector6d & eta, const Vector6d & nu,
  Matrix6d & J, Matrix6d & J_star)
{
  // Linearizing the 6DOF kinematics  η̇ = J(η) v.
  // See H2 and H∞ Designs, Lúcia Moreira. Equation (13)
  // η̇ ≈ J  ∆ν + J* ∆η
  // J = J(η0) * ∆ν
  // J* = ∂J(η)/∂η ν0
  // Stacking the pose η and the velocity ν
  VectorXdAD input(12);
  input.segment<6>(0) = eta.cast<CppAD::AD<double>>();
  input.segment<6>(6) = nu.cast<CppAD::AD<double>>();
  // Declare independent variables and starting recording
  CppAD::Independent(input);
  const Vector3dAD & euler_ad = input.segment<3>(3);
  const Vector6dAD & nu_ad = input.segment<6>(6);
  //  η̇ = J(η) v
  Vector6dAD eta_dot = nonlinear_kinematics(euler_ad, nu_ad);
  auto f = CppAD::ADFun<double>(input, VectorXdAD(eta_dot));
  VectorXd input_eq(12);
  input_eq.setZero();
  input_eq.segment<3>(6) = nu.segment<3>(0);
  Eigen::Matrix<double, 6, 12, Eigen::RowMajor> Jac;
  Eigen::Map<VectorXd> J_map(Jac.data(), Jac.size());
  J_map << f.Jacobian(VectorXd(input_eq));
  J_star = Jac.block<6, 6>(0, 0);
  J = Jac.block<6, 6>(0, 6);
}
// ================================================================
Matrix6d Plant::restoring_forces_jacobian(const Vector6d & eta)
{
  // Linearizing the restoring forces g(η)
  // G(t)≈ ∂g(η) / ∂η
  VectorXdAD _eta(6);
  _eta = eta.cast<CppAD::AD<double>>();
  CppAD::Independent(_eta);
  const Vector3dAD & euler = _eta.tail<3>();
  VectorXdAD g(6);
  g = nonlinear_restoring_forces(euler);
  auto f = CppAD::ADFun<double>(_eta, g);
  VectorXd eta_eq(6);
  eta_eq.setZero();
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> G;
  Eigen::Map<VectorXd> J_map(G.data(), G.size());
  J_map << f.Jacobian(eta_eq);
  return G;
}
// =========================================================================
Matrix6d Plant::coriolis_forces_jacobian(const Vector6d & nu)
{
  VectorXdAD _nu(6);
  _nu = nu.cast<CppAD::AD<double>>();
  CppAD::Independent(_nu);
  VectorXdAD fc(6);
  fc = nonlinear_coriolis_forces(_nu);
  auto f = CppAD::ADFun<double>(_nu, fc);
  VectorXd nu_eq(6);
  nu_eq.setZero();
  nu_eq.segment<3>(0) = nu.head<3>();
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> C;
  Eigen::Map<VectorXd> J_map(C.data(), C.size());
  J_map << f.Jacobian(nu_eq);
  return C;
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
