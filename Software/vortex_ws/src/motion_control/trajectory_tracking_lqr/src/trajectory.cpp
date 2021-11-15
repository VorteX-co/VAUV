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

#include "trajectory.hpp"

TrajectoryGenerator::TrajectoryGenerator() {}

void TrajectoryGenerator::generate(
  Vector6d & x_ref, Vector6d & x,
  int trajectory_type)
{
  if (trajectory_type == trajectory_types::translation) {
    // Create 3D parameters and solver
    ruckig::InputParameter<3> input;
    ruckig::Ruckig<3> otg;
    input.current_position = {x(0, 0), x(1, 0), x(2, 0)};
    input.current_velocity = {x(3, 0), x(4, 0), x(5, 0)};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {x_ref(0, 0), x_ref(1, 0), x_ref(2, 0)};
    input.target_velocity = {x_ref(3, 0), x_ref(4, 0), x_ref(5, 0)};
    // Target acceleration is 0 implies that we will stop at the end of the
    // trajectory
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {max_translation_vel_, max_translation_vel_,
      max_translation_vel_};
    input.max_acceleration = {max_translation_acc_, max_translation_acc_,
      max_translation_acc_};
    input.max_jerk = {max_translation_jerk_, max_translation_jerk_,
      max_translation_jerk_};
    input.min_velocity = {-max_translation_vel_, -max_translation_vel_,
      -max_translation_vel_};
    input.min_acceleration = {-max_translation_acc_, -max_translation_acc_,
      -max_translation_acc_};

    ruckig::Result result = otg.calculate(input, translation3D_trajectory_);

    if (result == ruckig::Result::ErrorInvalidInput) {
      std::cout << "Invalid input!" << std::endl;
    }
    translation3D_duration = translation3D_trajectory_.get_duration();
  } else if (trajectory_type == trajectory_types::rotation) {
    ruckig::InputParameter<3> input;
    input.current_position = {x(0, 0), x(1, 0), x(2, 0)};
    input.current_velocity = {x(3, 0), x(4, 0), x(5, 0)};
    input.current_acceleration = {0.0, 0.0, 0.0};
    input.target_position = {x_ref(0, 0), x_ref(1, 0), x_ref(2, 0)};
    input.target_velocity = {x_ref(3, 0), x_ref(4, 0), x_ref(5, 0)};
    input.target_acceleration = {0.0, 0.0, 0.0};
    input.max_velocity = {max_rotation_vel_, max_rotation_vel_,
      max_rotation_vel_};
    input.max_acceleration = {max_rotation_acc_, max_rotation_acc_,
      max_rotation_acc_};
    input.max_jerk = {max_rotation_jerk_, max_rotation_jerk_,
      max_rotation_jerk_};
    input.min_velocity = {-max_rotation_vel_, -max_rotation_vel_,
      -max_rotation_vel_};
    input.min_acceleration = {-max_rotation_acc_, -max_rotation_acc_,
      -max_rotation_acc_};
    ruckig::Ruckig<3> otg;
    //
    ruckig::Result result = otg.calculate(input, rotation3D_trajectory_);

    if (result == ruckig::Result::ErrorInvalidInput) {
      std::cout << "Invalid input!" << std::endl;
    }
    rotation3D_duration = rotation3D_trajectory_.get_duration();
  }
}
void TrajectoryGenerator::generate(
  Vector2d & x_ref, Vector2d & x,
  int trajectory_type)
{
  if (trajectory_types::translation) {
    ruckig::InputParameter<1> input;
    input.current_position = {x(0, 0)};
    input.current_velocity = {x(1, 0)};
    input.target_velocity = {x_ref(1, 0)};
    input.current_acceleration = {0.0};
    input.target_position = {x_ref(0, 0)};
    input.target_acceleration = {0.0};
    input.max_velocity = {max_translation_vel_};
    input.max_acceleration = {max_translation_acc_};
    input.max_jerk = {max_translation_jerk_};
    input.min_velocity = {-max_translation_vel_};
    input.min_acceleration = {-max_translation_acc_};
    ruckig::Ruckig<1> otg;
    ruckig::Result result = otg.calculate(input, translation1D_trajectory_);
    if (result == ruckig::Result::ErrorInvalidInput) {
      std::cout << "Invalid input!" << std::endl;
    }
    translation1D_duration = translation1D_trajectory_.get_duration();
  } else if (trajectory_types::rotation) {
    ruckig::InputParameter<1> input;
    input.current_position = {x(0, 0)};
    input.current_velocity = {x(1, 0)};
    input.current_acceleration = {0.0};
    input.target_position = {x_ref(0, 0)};
    input.target_velocity = {x_ref(1, 0)};
    input.target_acceleration = {0.0};
    input.max_velocity = {max_rotation_vel_};
    input.max_acceleration = {max_rotation_acc_};
    input.max_jerk = {max_rotation_jerk_};
    input.min_velocity = {-max_rotation_vel_};
    input.min_acceleration = {-max_rotation_acc_};
    ruckig::Ruckig<1> otg;
    ruckig::Result result = otg.calculate(input, rotation1D_trajectory_);
    if (result == ruckig::Result::ErrorInvalidInput) {
      std::cout << "Invalid input!" << std::endl;
    }
    rotation1D_duration = rotation1D_trajectory_.get_duration();
  }
}
Vector9d TrajectoryGenerator::get_translation3D_trajectory(double & t)
{
  std::array<double, 3> new_position, new_velocity, new_acceleration;
  translation3D_trajectory_.at_time(t, new_position, new_velocity,
    new_acceleration);
  Vector9d desired_state;
  desired_state << new_position[0], new_position[1], new_position[2],
    new_velocity[0], new_velocity[1], new_velocity[2], new_acceleration[0],
    new_acceleration[1], new_acceleration[2];
  return desired_state;
}
Vector9d TrajectoryGenerator::get_rotation3D_trajectory(double & t)
{
  std::array<double, 3> new_position, new_velocity, new_acceleration;
  rotation3D_trajectory_.at_time(t, new_position, new_velocity,
    new_acceleration);
  Vector9d desired_state;
  desired_state << new_position[0], new_position[1], new_position[2],
    new_velocity[0], new_velocity[1], new_velocity[2], new_acceleration[0],
    new_acceleration[1], new_acceleration[2];
  return desired_state;
}
Vector3d TrajectoryGenerator::get_translation1D_trajectory(double & t)
{
  std::array<double, 1> new_position, new_velocity, new_acceleration;
  translation1D_trajectory_.at_time(t, new_position, new_velocity,
    new_acceleration);
  Vector3d desired_state;
  desired_state << new_position[0], new_velocity[0], new_acceleration[0];
  return desired_state;
}
Vector3d TrajectoryGenerator::get_rotation1D_trajectory(double & t)
{
  Vector3d xdes;
  std::array<double, 1> new_position, new_velocity, new_acceleration;
  rotation1D_trajectory_.at_time(t, new_position, new_velocity,
    new_acceleration);
  Vector3d desired_state;
  desired_state << new_position[0], new_velocity[0], new_acceleration[0];
  return desired_state;
}
