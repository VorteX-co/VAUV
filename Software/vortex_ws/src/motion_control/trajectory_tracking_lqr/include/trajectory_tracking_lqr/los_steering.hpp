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

#ifndef TRAJECTORY_TRACKING_LQR__LOS_STEERING_HPP_
#define TRAJECTORY_TRACKING_LQR__LOS_STEERING_HPP_
#include <Eigen/Core>
#include <vector>

/**
 * @brief The ILOS class computes the desired
 * heading angle when the path is straight lines going through a desired
 * waypoints Reference: E. Børhaug, A. Pavlov and K. Y. Pettersen (2008).
 * Integral LOS Control for Path Following of Underactuated Marine Surface
 * Vessels in the presence of Constant Ocean Currents.
 */
class ILOS
{
public:
  /**
   * @brief  sets ILOS paramteres
   * @param delta, lookahead distance △ (m)
   *                R,      radius of acceptance  (m)
   *                dt,     sampling time              (s)
   *                kappa, postive gain paramtere for the integral error
   *                T,         yaw time constant    (s)
   */
  void set_params(
    const double & delta, const double & R, const double & dt,
    const double & kappa, const double & T);
  /**
   * @brief  sets the current target position
   * @param wpt  2D-waypoint
   */
  void setpoint(const Eigen::Vector2d & wpt);
  void setpoints(const std::vector<Eigen::Vector2d> & wpts);
  /**
   * @brief steering-law calculation
   * @param pose [x y yaw]
   * @return yaw_reference
   */
  double calculate_reference(const Eigen::Vector3d & planner_pose);

private:
  double dt_;          // sampling time
  double R_;           // Radius of the sphere of acceptance
  double delta_;       // lookahead distance = [1.5:2.5] * vehicle length
  double kappa_;       // Integral error gain > 0
  double T_;           // Yaw time constant
  double e_int_{0.0};  // Cross-track error integral
  std::vector<Eigen::Vector2d> wpts_;
  int k_{0};  // Active waypoint index
};
#endif  // TRAJECTORY_TRACKING_LQR__LOS_STEERING_HPP_
