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
#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_
#include <math.h>
#include "eigen3/Eigen/Dense"

namespace auv_guidance
{
class Waypoint
{
private:
  Eigen::Vector3d posI_, velI_, accelI_;  // Inertial Position, velocity, and
                                          // acceleration expressed in I-frame
  Eigen::Quaterniond quaternion_;         // Attitude wrt I-frame
  Eigen::Vector3d angVelB_;               // Angular velocity about B-frame axis

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Waypoint(
    const Eigen::Ref<const Eigen::Vector3d> & posI,
    const Eigen::Ref<const Eigen::Vector3d> & velI,
    const Eigen::Ref<const Eigen::Vector3d> & accelI,
    const Eigen::Quaterniond & quaternion,
    const Eigen::Ref<const Eigen::Vector3d> & angVelB);
  Eigen::Vector3d xI();
  Eigen::Vector3d yI();
  Eigen::Vector3d zI();
  Eigen::Vector3d posI();
  Eigen::Vector3d velI();
  Eigen::Vector3d accelI();
  Eigen::Quaterniond quaternion();
  Eigen::Vector3d angVelB();
};
}  // namespace auv_guidance

#endif  // WAYPOINT_HPP_
