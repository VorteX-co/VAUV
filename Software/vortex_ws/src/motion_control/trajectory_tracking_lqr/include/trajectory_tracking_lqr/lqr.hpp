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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "lin_alg_tools/care.h"
#include "lin_alg_tools/schur.h"

#ifndef TRAJECTORY_TRACKING_LQR__LQR_HPP_
#define TRAJECTORY_TRACKING_LQR__LQR_HPP_

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

class LQR
{
public:
  /*
   * @brief Constructor that initializes all parameters
   */
  LQR();
  /*
   * @brief LQR control action
   * @param state vector, desired state vector and acceleration feedforward
   */
  Vector6d action(Vector12d & x, Vector12d & xd, Vector6d & ffacc);

private:
  /*
   * @brief Compute Damping matrix D
   *
   */
  Matrix6d calculate_damping_matrix(Vector6d & nu);
  /*
   * @brief Compute Jacobian matrix J
   * @param euler attitude [rpy]
   */
  Matrix6d calculate_jacobian_matrix(Vector3d & euler);
  /*
   * @brief Compute Coriolis–Centripetal matrix C
   * @param nu  body velocity [NED]
   */
  Matrix6d calculate_coriolis_matrix(Vector6d & nu);
  /*
   * @brief Compute restoring forces and moments  g
   * @params euler attitude [rpy]
   */
  Vector6d calculate_restoring_forces(Vector3d & euler);
  /*
   * @brief Saturate the output forces
   * @param tau Control forces and moments
   */
  void saturate_control(Vector6d & tau);
  /*
   * @brief Saturate the state error vector to a certian value
   * @param delta (x-xd)
   */
  void saturate_error(Vector12d & delta);
  /*
   * @brief ENU <> NED
   * @param x any 12D vector
   */
  void to_SNAME(Vector12d & x);
  /*
   * @brief Compute Rotation matrix from Body to inertial frame
   * @param euler attitde [φ,θ,ψ]
   */
  Matrix3d RBtoI(Vector3d & euler);
  /*
   * @brief Compute Translation matrix from Body to inertial frame
   * @param euler attitde [φ,θ,ψ]
   */
  Matrix3d TBtoI(Vector3d & euler);
  /*
   * @brief Compute Translation matrix from Body to inertial frame
   * @param euler attitde [φ,θ,ψ]
   */
  Matrix3d TItoB(Vector3d & euler);

  // Algebraic Riccati equation solver [12 states, 6 controls]
  CareSolver<12, 6> care_solver_;
  double mass_{0.0};
  double volume_{0.0};
  // Origin to CB [xb ,yb ,zb ]
  Vector3d rb_;
  // Origin to CG [xg ,yg ,zg ]
  Vector3d rg_;
  // Inertia matrix
  Matrix3d Ib_;
  // Rigid-body mass matrix
  Matrix6d Mr_;
  // Added-mass matrix
  Matrix6d Ma_;
  // Total mass matrix
  Matrix6d M_;
  // Linear Damping  matrix
  Matrix6d DL_;
  // NonLinear Damping matrix
  Matrix6d DNL_;
  // The control forces weighting matrix
  Matrix6d R_;
  // The state weighting matrix
  Matrix12d Q_;
};

#endif  // TRAJECTORY_TRACKING_LQR__LQR_HPP_
