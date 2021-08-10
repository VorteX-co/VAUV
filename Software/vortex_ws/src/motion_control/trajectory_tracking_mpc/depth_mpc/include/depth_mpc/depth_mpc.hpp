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
#ifndef DEPTH_MPC__DEPTH_MPC_HPP_
#define DEPTH_MPC__DEPTH_MPC_HPP_
#include <Eigen/Core>
#include <map>
#include <vector>
#include <string>
namespace Depth
{
class MPC
{
public:
  MPC();
  ~MPC();
  // Return the first actuatotions.
  vector<double> Solve(
    const Eigen::VectorXd & x0,
    const Eigen::Vector2d & reference_state,
    const double & reference_acc);
  double prev_Fz{0};
  double prev_My{0};
  /**
   * @brief set the MPC required parameters
   * @param params
   */
  void set_params(const std::map<string, double> & params);

private:
  std::map<string, double> depth_params;
};
}  // namespace Depth
#endif  // DEPTH_MPC__DEPTH_MPC_HPP_
