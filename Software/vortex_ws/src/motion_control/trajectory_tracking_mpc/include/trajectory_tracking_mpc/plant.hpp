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

#ifndef TRAJECTORY_TRACKING_MPC__PLANT_HPP_
#define TRAJECTORY_TRACKING_MPC__PLANT_HPP_

#include <cppad/example/cppad_eigen.hpp>
#include <Eigen/Dense>
#include "eigen_headers.hpp"


using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

using Vector12dAD = Eigen::Matrix<CppAD::AD<double>, 12, 1>;
using Matrix12dAD = Eigen::Matrix<CppAD::AD<double>, 12, 12>;
using Vector6dAD = Eigen::Matrix<CppAD::AD<double>, 6, 1>;
using Matrix6dAD = Eigen::Matrix<CppAD::AD<double>, 6, 6>;
using Vector3dAD = Eigen::Matrix<CppAD::AD<double>, 3, 1>;
using Matrix3dAD = Eigen::Matrix<CppAD::AD<double>, 3, 3>;

using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using VectorXdAD = Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>;

class Plant
{
public:
  // Model parameters initialization
  void initialize(
    const double & m, const double & volume, const Vector6d & Ib,
    const Vector3d & r_cob, const Vector3d & r_cog,
    const Vector6d & Ma, const Vector6d & Dlinear,
    const Vector6d & Dquad);

  // Compute AUV motion model, x_next = integration of ẋ = f(x, u)
  Vector12dAD nonlinear_update(const Vector12dAD & x, const Vector6dAD & u);
  // Calculating the body velocities nu given the  inertial velocities η̇
  // nu =  J^-1 * η̇
  Vector6d kinematics_inverse(
    const Vector3d & euler,
    const Vector6d & eta_dot);

  // AUV 6DOF Kinematics, η̇ =  J * nu
  // euler is the attitude rpy and nu is the body velocities
  // calculating the inertial velocities η̇ given the body velocities nu
  Vector6dAD nonlinear_kinematics(
    const Vector3dAD & euler,
    const Vector6dAD & nu);

private:
  //  Compute Damping forces and moments D(nu) nu
  Vector6dAD nonlinear_damping_forces(const Vector6dAD & nu);
  // Compute Coriolis–Centripetal forces and moments C(nu) nu
  Vector6dAD nonlinear_coriolis_forces(const Vector6dAD & nu);
  // Compute restoring forces and moments  g(η)
  Vector6dAD nonlinear_restoring_forces(const Vector3dAD & euler);

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
#endif  // TRAJECTORY_TRACKING_MPC__PLANT_HPP_
