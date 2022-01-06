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
#ifndef MOTION_CONTROL__PLANT_HPP_
#define MOTION_CONTROL__PLANT_HPP_
#include <cppad/cppad.hpp>
#include "eigen_headers.hpp"

#pragma once
class Plant
{
public:
  // Model parameters initialization
  void initialize(
    const double & m, const double & volume, const Vector6d & Ib,
    const Vector3d & r_cob, const Vector3d & r_cog,
    const Vector6d & Ma, const Vector6d & Dlinear,
    const Vector6d & Dquad);

  // Compute AUV motion model, ẋ = f(x, u)
  Vector12dAD nonlinear_update(const Vector12dAD & x, const Vector6dAD & u);

  // Linearize f(x, u) and obtain Jacobians A and B,  ẋ = A x + B u
  void linearize(
    const Vector12d & x, const Vector6d & u, Matrix12d & A,
    Eigen::Matrix<double, 12, 6> & B);

private:
  // AUV 6DOF Kinematics, η̇ =  J * nu
  // euler is the attitude rpy and nu is the body velocities
  Vector6dAD nonlinear_kinematics(
    const Vector3dAD & euler,
    const Vector6dAD & nu);
  //  Compute Damping forces and moments D(nu) nu
  Vector6dAD nonlinear_damping_forces(const Vector6dAD & nu);
  // Compute Coriolis–Centripetal forces and moments C(nu) nu
  Vector6dAD nonlinear_coriolis_forces(const Vector6dAD & nu);
  // Compute restoring forces and moments  g(η)
  Vector6dAD nonlinear_restoring_forces(const Vector3dAD & euler);

  // Linearization of Kinematics
  void kinematics_jacobian(
    const Vector6d & eta, const Vector6d & nu, Matrix6d & J,
    Matrix6d & J_star);
  // Linearization of Damping forces
  Matrix6d damping_forces_jacobian(const Vector6d & nu);
  // Linearization of Coriolis–Centripetal forces
  Matrix6d coriolis_forces_jacobian(const Vector6d & nu);
  //  Linearization of restoring forces
  Matrix6d restoring_forces_jacobian(const Vector6d & eta);

  // Compute Rotation matrix from Body to inertial frame
  Matrix3dAD RBtoI(const Vector3dAD & euler);
  // Compute Transformation matrix from Body to inertial frame
  Matrix3dAD TBtoI(const Vector3dAD & euler);
  // Compute Transformation matrix from Body to inertial frame
  Matrix3dAD TItoB(const Vector3dAD & euler);
  // skew-symmetric matrix using CppAD::AD<double>
  Matrix3dAD skew_ad(const Vector3dAD & v);
  // using double
  Matrix3d skew(const Vector3d & v);

  CppAD::AD<double> mass_{0.0};
  CppAD::AD<double> volume_{0.0};
  // Origin to CB [xb, yb, zb ]
  Vector3dAD r_cob_;
  // Origin to CG [xg, yg, zg ]
  Vector3dAD r_cog_;
  // Total mass matrix
  Matrix6dAD M_ad_;
  // Total mass matrix
  Matrix6d M_;
  // Linear Damping  matrix
  Vector6dAD Dlinear_;
  // NonLinear Damping matrix
  Vector6dAD Dquad_;
};
#endif  // MOTION_CONTROL__PLANT_HPP_
