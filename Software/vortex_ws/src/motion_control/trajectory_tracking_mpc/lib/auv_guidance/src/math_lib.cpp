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
#include "math_lib.hpp"

namespace auv_core
{
namespace math_lib
{
/**
 * @param x (double) Value to take the sign of
 * \brief Return the sign of x. Return 0 if x == 0.
 */
int sign(double x)
{
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  }
  return 0;
}

/**
 * @param x (float) Value to take the sign of
 * \brief Return the sign of x. Return 0 if x == 0.
 */
int sign(float x)
{
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  }
  return 0;
}

/**
 * @param x (int) Value to take the sign of
 * \brief Return the sign of x. Return 0 if x == 0.
 */
int sign(int x)
{
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  }
  return 0;
}

/**
 * @param mat Input matrix
 * \brief Return the element-wise sign of mat
 */
Eigen::MatrixXf sign(const Eigen::Ref<const Eigen::MatrixXf> & mat)
{
  Eigen::MatrixXf signMat(mat.rows(), mat.cols());
  signMat.setZero();

  for (int i = 0; i < mat.rows(); i++) {
    for (int j = 0; j < mat.cols(); j++) {
      if (mat(i, j) > 0) {
        signMat(i, j) = 1;
      } else if (mat(i, j) < 0) {
        signMat(i, j) = -1;
      }
    }
  }
  return signMat;
}

/**
 * @param mat Input matrix
 * \brief Return the element-wise sign of mat
 */
Eigen::MatrixXd sign(const Eigen::Ref<const Eigen::MatrixXd> & mat)
{
  Eigen::MatrixXd signMat(mat.rows(), mat.cols());
  signMat.setZero();

  for (int i = 0; i < mat.rows(); i++) {
    for (int j = 0; j < mat.cols(); j++) {
      if (mat(i, j) > 0) {
        signMat(i, j) = 1;
      } else if (mat(i, j) < 0) {
        signMat(i, j) = -1;
      }
    }
  }
  return signMat;
}
}  // namespace math_lib
}  // namespace auv_core
