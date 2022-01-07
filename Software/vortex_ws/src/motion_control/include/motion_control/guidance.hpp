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

#ifndef MOTION_CONTROL__GUIDANCE_HPP_
#define MOTION_CONTROL__GUIDANCE_HPP_

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>
#include <tuple>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

class Guidance
{
public:
  void set_params(
    const Vector3d & translation_constraints,
    const Vector3d & rotation_constraints,
    const double & radius_of_acceptance);
  void los_setpoint(const Vector2d & p);
  void los_steering(const Vector2d & p, double & psi_des, double & r_des);
  void generate_trajectory(const Vector6d & state, const Vector6d & reference);
  std::tuple<Vector6d, Vector6d, Vector6d> evaluate_desired_trajectory(
    const double & t);

private:
  ruckig::Trajectory<6> desired_trajectory_;
  Vector3d translation_constraints_;
  Vector3d rotation_constraints_;
  Vector2d prev_goal_;
  Vector2d goal_;
  double dt_;
  double radius_of_acceptance_;
  double kappa_;
  double delta_;
  double e_int_{0.0};
  double psi_des_smooth_{0.0};
};
#endif  // MOTION_CONTROL__GUIDANCE_HPP_
