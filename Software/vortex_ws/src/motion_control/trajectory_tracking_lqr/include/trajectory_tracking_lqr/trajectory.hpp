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

class TrajectoryGenerator
{
public:
  TrajectoryGenerator();
  /*
   * @brief Generate a 3D translation trajectory
   * @param x_ref reference [xd,yd,zd,ud,vd,wd] x state [x,y,z,u,v,w]
   */
  void generate(Vector6d x_ref, Vector6d x);
  /*
   * @brief Generate a 1D rotation trajectory for any attitude angle
   * @param x_ref reference [desired angular position, velocity] x state
   * [current angular position, velocity]
   */
  void generate(Vector2d x_ref, Vector2d x);
  /*
   * @brief Evaluate the generated trajectory at current time
   * @param t  time in seconds
   */
  Vector9d get_translation_trajectory(double t);
  Vector3d get_rotation_trajectory(double t);
  // Duration in seconds of the generated translation trajectory
  double translation_duration{0.0};
  // Duration in seconds of the generated rotation trajectory
  double rotation_duration{0.0};

private:
  // Create 3D translation input parameters, solver and output trajectory
  ruckig::InputParameter<3> translation_input;
  ruckig::Ruckig<3> translation_otg;
  ruckig::Trajectory<3> translation_trajectory;
  // Create 1D rotation input parameters, solver and output trajectory
  ruckig::InputParameter<1> rotation_input;
  ruckig::Ruckig<1> rotation_otg;
  ruckig::Trajectory<1> rotation_trajectory;
  // Set different constraints
  double max_translation_vel{0.6};
  double max_rotation_vel{0.45};
  double max_translation_acc{0.20};
  double max_rotation_acc{0.175};
  double max_translation_jerk{0.0002};
  double max_rotation_jerk{0.0001};
};

#endif  // TRAJECTORY_TRACKING_LQR__TRAJECTORY_HPP_
