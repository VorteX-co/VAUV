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

#include <eigen3/Eigen/Core>
#include <ruckig/ruckig.hpp>

#ifndef TRAJECTORY_TRACKING_LQR__TRAJECTORY_HPP_
#define TRAJECTORY_TRACKING_LQR__TRAJECTORY_HPP_

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

enum trajectory_types { translation = 0, rotation = 1 };
class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  /*
   * @brief Generate a 3DOF  trajectory with velocity and acceleration profiles
   * @param x_ref: reference [x1_ref,x2_ref,x3_ref,v1_ref,v2_ref,v3_ref]
   *                x: state [x1,x2,x3,v1,v2,v3]
   */
  void generate(Vector6d & x_ref, Vector6d & x, int trajectory_type);
  /*
   * @brief Generate a 1D  trajectory
   * @param x_ref [reference position, reference velocity]
   *                x state [current position, current velocity]
   */
  void generate(Vector2d & x_ref, Vector2d & x, int trajectory_type);
  /*
   * @brief Evaluate the generated trajectory at current time
   * @param t  time in seconds
   */
  Vector9d get_translation3D_trajectory(double & t);
  Vector9d get_rotation3D_trajectory(double & t);
  Vector3d get_translation1D_trajectory(double & t);
  Vector3d get_rotation1D_trajectory(double & t);
  // Duration in seconds of the generated translation trajectory
  double translation3D_duration{0.0};
  double translation1D_duration{0.0};
  // Duration in seconds of the generated rotation trajectory
  double rotation3D_duration{0.0};
  double rotation1D_duration{0.0};

private:
  // The generated trajectories
  ruckig::Trajectory<3> translation3D_trajectory_;
  ruckig::Trajectory<3> rotation3D_trajectory_;
  ruckig::Trajectory<1> translation1D_trajectory_;
  ruckig::Trajectory<1> rotation1D_trajectory_;
  // Different constraints
  double max_translation_vel_{0.55};
  double max_rotation_vel_{0.4};
  double max_translation_acc_{0.3};
  double max_rotation_acc_{0.2};
  double max_translation_jerk_{0.0002};
  double max_rotation_jerk_{0.0001};
};

#endif  // TRAJECTORY_TRACKING_LQR__TRAJECTORY_HPP_
