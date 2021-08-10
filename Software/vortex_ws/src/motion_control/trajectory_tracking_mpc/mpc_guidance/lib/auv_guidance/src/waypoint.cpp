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
#include "../include/waypoint.hpp"

namespace auv_guidance
{
Waypoint::Waypoint(
  const Eigen::Ref<const Eigen::Vector3d> & posI,
  const Eigen::Ref<const Eigen::Vector3d> & velI,
  const Eigen::Ref<const Eigen::Vector3d> & accelI,
  const Eigen::Quaterniond & quaternion,
  const Eigen::Ref<const Eigen::Vector3d> & angVelB)
{
  posI_ = posI;
  velI_ = velI;
  accelI_ = accelI;
  quaternion_ = quaternion.normalized();
  angVelB_ = angVelB;
}

Eigen::Vector3d Waypoint::xI()
{
  Eigen::Vector3d xI = Eigen::Vector3d::Zero();
  xI << posI_(0), velI_(0), accelI_(0);
  return xI;
}

Eigen::Vector3d Waypoint::yI()
{
  Eigen::Vector3d yI = Eigen::Vector3d::Zero();
  yI << posI_(1), velI_(1), accelI_(1);
  return yI;
}

Eigen::Vector3d Waypoint::zI()
{
  Eigen::Vector3d zI = Eigen::Vector3d::Zero();
  zI << posI_(2), velI_(2), accelI_(2);
  return zI;
}

Eigen::Vector3d Waypoint::posI() {return posI_;}

Eigen::Vector3d Waypoint::velI() {return velI_;}

Eigen::Vector3d Waypoint::accelI() {return accelI_;}

Eigen::Quaterniond Waypoint::quaternion() {return quaternion_;}

Eigen::Vector3d Waypoint::angVelB() {return angVelB_;}
}  // namespace auv_guidance
