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
#ifndef TGEN_LIMITS_HPP_
#define TGEN_LIMITS_HPP_

#include <math.h>

namespace auv_guidance
{
class TGenLimits
{
private:
  // SI units - m, m/s, m/s^2, m/s^3
  double maxXYDistance_, maxZDistance_, maxPathInclination_, closingTolXYZ_,
    closingTolRot_;
  double maxXVel_, maxYVel_, maxZVel_, maxRotVel_;
  double maxXAccel_, maxYAccel_, maxZAccel_, maxRotAccel_;
  double xyzJerk_, xyzClosingJerk_;
  double rotJerk_, rotClosingJerk_;

public:
  TGenLimits(
    double maxXYDistance, double maxZDistance,
    double maxPathInclination, double closingTolXYZ,
    double closingTolRot, double maxXVel, double maxYVel,
    double maxZVel, double maxRotVel, double maxXAccel,
    double maxYAccel, double maxZAccel, double maxRotAccel,
    double xyzJerk, double xyzClosingJerk, double rotJerk,
    double rotClosingJerk);
  double maxXYDistance();
  double maxZDistance();
  double closingTolXYZ();
  double closingTolRot();
  double maxPathInclination();
  double maxXVel();
  double maxYVel();
  double maxZVel();
  double maxRotVel();
  double maxXAccel();
  double maxYAccel();
  double maxZAccel();
  double maxRotAccel();
  double xyzJerk(double distance);
  double rotJerk(double distance);
};
}  // namespace auv_guidance

#endif  // TGEN_LIMITS_HPP_
