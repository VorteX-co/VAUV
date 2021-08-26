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
#include "tgen_limits.hpp"

namespace auv_guidance
{
TGenLimits::TGenLimits(
  double maxXYDistance, double maxZDistance,
  double maxPathInclination, double closingTolXYZ,
  double closingTolRot, double maxXVel, double maxYVel,
  double maxZVel, double maxRotVel, double maxXAccel,
  double maxYAccel, double maxZAccel, double maxRotAccel,
  double xyzJerk, double xyzClosingJerk, double rotJerk,
  double rotClosingJerk)
{
  maxXYDistance_ = fabs(maxXYDistance);
  maxZDistance_ = fabs(maxZDistance);
  closingTolXYZ_ = fabs(closingTolXYZ);
  closingTolRot_ = fabs(closingTolRot);

  maxXVel_ = fabs(maxXVel);
  maxYVel_ = fabs(maxYVel);
  maxZVel_ = fabs(maxZVel);
  maxRotVel_ = fabs(maxRotVel);

  maxXAccel_ = fabs(maxXAccel);
  maxYAccel_ = fabs(maxYAccel);
  maxZAccel_ = fabs(maxZAccel);
  maxRotAccel_ = fabs(maxRotAccel);

  xyzJerk_ = fabs(xyzJerk);
  xyzClosingJerk_ = fabs(xyzClosingJerk);
  rotJerk_ = fabs(rotJerk);
  rotClosingJerk_ = fabs(rotClosingJerk);

  maxPathInclination_ = fabs(maxPathInclination);
  if (maxPathInclination_ > M_PI / 2.0) {
    maxPathInclination_ = M_PI / 2.0;
  }
}

double TGenLimits::maxXYDistance() {return maxXYDistance_;}

double TGenLimits::maxZDistance() {return maxZDistance_;}

double TGenLimits::closingTolXYZ() {return closingTolXYZ_;}

double TGenLimits::closingTolRot() {return closingTolRot_;}

double TGenLimits::maxPathInclination() {return maxPathInclination_;}

double TGenLimits::maxXVel() {return maxXVel_;}

double TGenLimits::maxYVel() {return maxYVel_;}

double TGenLimits::maxZVel() {return maxZVel_;}

double TGenLimits::maxRotVel() {return maxRotVel_;}

double TGenLimits::maxXAccel() {return maxXAccel_;}

double TGenLimits::maxYAccel() {return maxYAccel_;}

double TGenLimits::maxZAccel() {return maxZAccel_;}

double TGenLimits::maxRotAccel() {return maxRotAccel_;}

double TGenLimits::xyzJerk(double distance)
{
  return (distance > closingTolXYZ_) ? xyzJerk_ : xyzClosingJerk_;
}

double TGenLimits::rotJerk(double distance)
{
  return (distance > closingTolRot_) ? rotJerk_ : rotClosingJerk_;
}

}  // namespace auv_guidance
