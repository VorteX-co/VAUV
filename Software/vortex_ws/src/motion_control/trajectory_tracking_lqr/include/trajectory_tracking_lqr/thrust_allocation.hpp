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
#ifndef TRAJECTORY_TRACKING_LQR__THRUST_ALLOCATION_HPP_
#define TRAJECTORY_TRACKING_LQR__THRUST_ALLOCATION_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class Allocator
{
public:
  // =========================================================================
  void set_params(
    const double & k, const MatrixXd & thrusters,
    const Vector4d & coeff_left, const Vector4d & coeff_right);
  Eigen::VectorXd wrench_to_pwm_thrusters(const Eigen::VectorXd & wrench);

  // =========================================================================

private:
  /**
   * @brief thrust_configuration compute thruster contribution for every DOF
   * @param Tpose thruster position and orientation w.r.t COG [x,y,z,r,p,y]
   * @return 6DOF thrust configuration vector
   */
  Vector6d thrust_configuration(const Vector6d & Tpose);

  double k_;
  MatrixXd thrusters_;
  Vector4d coeff_left_;
  Vector4d coeff_right_;
  // B† Moore–Penrose pseudo-inverse of B
  // B = T K     .. where T is the thruster configuration matrix
  // K is the thust coefficient matrix
  MatrixXd Bpinv_;
};
#endif  // TRAJECTORY_TRACKING_LQR__THRUST_ALLOCATION_HPP_
