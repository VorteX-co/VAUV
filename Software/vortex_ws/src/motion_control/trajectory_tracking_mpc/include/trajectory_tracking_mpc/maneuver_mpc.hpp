// Copyright 2021 VorteX-co
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
#ifndef TRAJECTORY_TRACKING_MPC__MANEUVER_MPC_HPP_
#define TRAJECTORY_TRACKING_MPC__MANEUVER_MPC_HPP_
#include <Eigen/Core>
#include <map>
#include <vector>
#include <string>
using std::vector;
using std::string;
namespace Maneuver
{
/**
 * @brief MPC class is an implementation of the receding horizon control for
 * planner AUV motion
 */
class MPC
{
public:
  MPC();
  ~MPC();
  /**
   * @brief  solves the optimization problem and outputs the control-law
   * @param x0 the controller state
   * @param reference_state the commanded state
   * @param reference_acc feedforward acceleration
   * @return the first set of actuations
   */
  vector<double> Solve(
    const Eigen::VectorXd & x0,
    const Eigen::VectorXd & reference_state,
    const Eigen::Vector3d & reference_acc);
  /**
   * @brief set the MPC required parameters
   * @param params
   */
  void set_params(const std::map<string, double> & params);
  double prev_Fx{0};
  double prev_Fy{0};
  double prev_Mz{0};

private:
  std::map<string, double> maneuver_params;
};
}  // namespace Maneuver
#endif  // TRAJECTORY_TRACKING_MPC__MANEUVER_MPC_HPP_
