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
  Vector6d action(Vector12d x, Vector12d xd, Vector6d ffacc);

private:
  /*
   * @brief Compute Damping matrix D
   */
  Matrix6d calculate_damping_matrix();
  /*
   * @brief Compute Jacobian matrix J
   */
  Matrix6d calculate_jacobian_matrix();
  /*
   * @brief Compute Coriolis–Centripetal matrix C
   */
  Matrix6d calculate_coriolis_matrix();
  /*
   * @brief Compute restoring forces and moments  g
   */
  Vector6d calculate_restoring_forces();
  /*
   * @brief Saturate the output forces
   * @param tau Control forces and moments
   */
  Vector6d saturate_control(Vector6d tau);
  /*
   * @brief Saturate the state error vector to a certian value
   * @param delta (x-xd)
   */
  Vector12d saturate_error(Vector12d delta);
  /*
   * @brief ENU <> NED
   * @param x any 6D vector
   */
  void to_SNAME(Vector6d & x);
  /*
   * @brief Compute Rotation matrix from Body to inertial frame
   * @param euler attitde [ψ,θ,ψ]
   */
  Matrix3d RBtoI(Vector3d euler);
  /*
   * @brief Compute Translation matrix from Body to inertial frame
   * @param euler attitde [ψ,θ,ψ]
   */
  Matrix3d TBtoI(Vector3d euler);
  /*
   * @brief Compute Translation matrix from Body to inertial frame
   * @param euler attitde [ψ,θ,ψ]
   */
  Matrix3d TItoB(Vector3d euler);
  // Algebraic Riccati equation solver
  CareSolver<12, 6> care_solver;
  double mass{0.0};
  double volume{0.0};
  // pose
  Vector6d eta;
  // velocity
  Vector6d nu;
  // Origin to CB [xb ,yb ,zb ]
  Vector3d rb;
  // Origin to CG [xg ,yg ,zg ]
  Vector3d rg;
  // Inertia matrix
  Matrix3d Ib;
  // Rigid-body mass matrix
  Matrix6d Mr;
  // Added-mass matrix
  Matrix6d Ma;
  // Total mass matrix
  Matrix6d M;
  // Linear Damping  matrix
  Matrix6d DL;
  // NonLinear Damping matrix
  Matrix6d DNL;
  // The control forces weighting matrix
  Matrix6d R;
  // The state weighting matrix
  Matrix12d Q;
};

#endif  // TRAJECTORY_TRACKING_LQR__LQR_HPP_
