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

#ifndef MOTION_PLANNING__LOCAL_PLANNING__STEERING_HPP_
#define MOTION_PLANNING__LOCAL_PLANNING__STEERING_HPP_

#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

class Steering
{
public:
  Steering() {}
  void set_params(
    const double & radius_of_acceptance, const double & dt,
    const double & kappa, const double & delta);
  void setpoint(const Vector2d & p);
  void compute_heading(const Vector2d & position, double & psi_des, double & r_des);

private:
  Vector2d prev_goal_;
  Vector2d goal_;
  double dt_;
  double radius_of_acceptance_;
  double kappa_;
  double delta_;
  double e_int_{0.0};
  double psi_des_smooth_{0.0};
};
#endif  // MOTION_PLANNING__LOCAL_PLANNING__STEERING_HPP_
