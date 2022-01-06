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
#ifndef MOTION_CONTROL__EIGEN_HEADERS_HPP_
#define MOTION_CONTROL__EIGEN_HEADERS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cppad/example/cppad_eigen.hpp>
#include <eigen3/Eigen/Eigen>

using Vector18d = Eigen::Matrix<double, 18, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Vector12i = Eigen::Matrix<int, 12, 1>;
using Vector6i = Eigen::Matrix<int, 6, 1>;

using Vector12dAD = Eigen::Matrix<CppAD::AD<double>, 12, 1>;
using Matrix12dAD = Eigen::Matrix<CppAD::AD<double>, 12, 12>;
using Vector6dAD = Eigen::Matrix<CppAD::AD<double>, 6, 1>;
using Matrix6dAD = Eigen::Matrix<CppAD::AD<double>, 6, 6>;
using Vector3dAD = Eigen::Matrix<CppAD::AD<double>, 3, 1>;
using Matrix3dAD = Eigen::Matrix<CppAD::AD<double>, 3, 3>;
using VectorXdAD = Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, 1>;
using MatrixXdAD =
  Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic>;

#endif  // MOTION_CONTROL__EIGEN_HEADERS_HPP_
