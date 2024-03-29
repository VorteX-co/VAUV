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
#ifndef EIGEN_TYPEDEFS_HPP_
#define EIGEN_TYPEDEFS_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

namespace auv_core
{
// Drag Matrix
typedef Eigen::Matrix<double, 6, 2> Matrix62d;

// Thruster matrices
typedef Eigen::Matrix<double, 5, 8> Matrix58d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

// State Space Control Matrices
typedef Eigen::Matrix<double, 18, 18> Matrix18d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 18, 8> Matrix18x8d;
typedef Eigen::Matrix<double, 12, 8> Matrix12x8d;
typedef Eigen::Matrix<double, 8, 18> Matrix8x18d;
typedef Eigen::Matrix<double, 8, 12> Matrix8x12d;

// Vectors
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 18, 1> Vector18d;

}  // namespace auv_core

#endif  // EIGEN_TYPEDEFS_HPP_
