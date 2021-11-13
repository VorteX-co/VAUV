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

void TrajectoryGenerator::generate(Vector6d x_ref, Vector6d x)
{
  translation_input.current_position = {x(0, 0), x(1, 0), x(2, 0)};
  translation_input.current_velocity = {x(3, 0), x(4, 0), x(5, 0)};
  translation_input.current_acceleration = {0.0, 0.0, 0.0};
  translation_input.target_position = {x_ref(0, 0), x_ref(1, 0), x_ref(2, 0)};
  translation_input.target_velocity = {x_ref(3, 0), x_ref(4, 0), x_ref(5, 0)};

  translation_input.target_acceleration = {0.0, 0.0, 0.0};
  translation_input.max_velocity = {max_translation_vel, max_translation_vel,
    max_translation_vel};
  translation_input.max_acceleration = {
    max_translation_acc, max_translation_acc, max_translation_acc};
  translation_input.max_jerk = {max_translation_jerk, max_translation_jerk,
    max_translation_jerk};
  translation_input.min_velocity = {-max_translation_vel, -max_translation_vel,
    -max_translation_vel};
  translation_input.min_acceleration = {
    -max_translation_acc, -max_translation_acc, -max_translation_acc};
  ruckig::Result result =
    translation_otg.calculate(translation_input, translation_trajectory);
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Invalid input!" << std::endl;
  }
  translation_duration = translation_trajectory.get_duration();
}
void TrajectoryGenerator::generate(Vector2d x_ref, Vector2d x)
{
  rotation_input.current_position = {x(0, 0)};
  rotation_input.current_velocity = {x(1, 0)};
  rotation_input.current_acceleration = {0.0};
  rotation_input.target_position = {x_ref(0, 0)};
  rotation_input.target_velocity = {x_ref(1, 0)};
  rotation_input.target_acceleration = {0.0};
  rotation_input.max_velocity = {max_rotation_vel};
  rotation_input.max_acceleration = {max_rotation_acc};
  rotation_input.max_jerk = {max_rotation_jerk};
  rotation_input.min_velocity = {-max_rotation_vel};
  rotation_input.min_acceleration = {-max_rotation_acc};
  ruckig::Result result =
    rotation_otg.calculate(rotation_input, rotation_trajectory);
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Invalid input!" << std::endl;
  }
  rotation_duration = rotation_trajectory.get_duration();
}
Vector9d TrajectoryGenerator::get_translation_trajectory(double t)
{
  Vector9d xdes;
  std::array<double, 3> new_position, new_velocity, new_acceleration;
  translation_trajectory.at_time(t, new_position, new_velocity,
    new_acceleration);
  xdes << new_position[0], new_position[1], new_position[2], new_velocity[0],
    new_velocity[1], new_velocity[2], new_acceleration[0],
    new_acceleration[1], new_acceleration[2];
  return xdes;
}
Vector3d TrajectoryGenerator::get_rotation_trajectory(double t)
{
  Vector3d xdes;
  std::array<double, 1> new_position, new_velocity, new_acceleration;
  rotation_trajectory.at_time(t, new_position, new_velocity, new_acceleration);
  xdes << new_position[0], new_velocity[0], new_acceleration[0];
  return xdes;
}
