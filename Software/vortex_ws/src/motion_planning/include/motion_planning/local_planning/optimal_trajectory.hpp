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

#ifndef MOTION_PLANNING__LOCAL_PLANNING__OPTIMAL_TRAJECTORY_HPP_
#define MOTION_PLANNING__LOCAL_PLANNING__OPTIMAL_TRAJECTORY_HPP_

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>
#include <tuple>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

class Trajectory
{
public:
  Trajectory();

  void set_params(
    const Vector3d & translation_constraints,
    const Vector3d & rotation_constraints);

  void generate_trajectory(const Vector12d & state, const Vector12d & reference);

  void evaluate_generated_trajectory(
    const double & t, Vector6d & pos,
    Vector6d & vel, Vector6d & acc);

private:
  ruckig::Trajectory<6> desired_trajectory_;
  Vector3d translation_constraints_;
  Vector3d rotation_constraints_;
};
#endif  // MOTION_PLANNING__LOCAL_PLANNING__OPTIMAL_TRAJECTORY_HPP_
