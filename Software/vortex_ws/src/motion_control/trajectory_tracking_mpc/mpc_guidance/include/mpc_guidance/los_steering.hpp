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
#ifndef MPC_GUIDANCE__LOS_STEERING_HPP_
#define MPC_GUIDANCE__LOS_STEERING_HPP_
#include <Eigen/Core>
#include <vector>

/**
 * @brief The LOS class is an implementation of lookahead-based steering law
 *  Reference: Handbook of Marine Craft Hydrodynamics and Motion Control, Thor
 * I. Fossen.
 */
class LOS
{
public:
  /**
   * @brief  sets the current target position
   * @param wp  2D-waypoint
   */
  void setpoint(const Eigen::Vector2d & wp);
  /**
   * @brief update the local state variables
   * @param pose [x y yaw]
   * @param vel [u v r]
   */
  void update_state(const Eigen::Vector3d & pose, const Eigen::Vector3d & vel);
  /**
   * @brief steering-law calculation
   */
  void calculate_reference();
  /**
   * @brief  calculate a feasible reference
   */
  void smooth_reference();
  double rd{0.0};
  double rdd{0.0};
  double yaw_des{0.0};
  bool inCircle{false};

private:
  double dt{0.055};   // sampling time
  double R{0.2};      // Radius of the sphere of acceptance
  double delta{4.0};  // lookahead distance = [1.5:2.5] * vehicle length
  Eigen::Vector3d pose_state{0.0, 0.0, 0.0};  // x y yaw
  Eigen::Vector3d vel_state{0.0, 0.0, 0.0};   // u v r
  Eigen::Vector2d goal{0.0, 0.0};
  Eigen::Vector2d prev_goal{0.0, 0.0};
  double yaw_ref{0.0};
  double yaw_ref_prev{0.0};
  double yaw_ref_prev_prev{0.0};
  double rd_prev{0.0};
  double r_ref{0.0};
  double yaw_des_prev{0.0};
  double yaw_des_prev_prev{0.0};
  void InCircle();
};
#endif  // MPC_GUIDANCE__LOS_STEERING_HPP_
