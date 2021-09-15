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
#ifndef AUV_STRUCTS_HPP_
#define AUV_STRUCTS_HPP_

#include "eigen_typedefs.hpp"

namespace auv_core
{

/**
 * AUV model parameters
 */
typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double mass;          // [kg]
  double Fg;            // [N]
  double Fb;            // [N]
  Eigen::Vector3d cob;  // Center of buoyancy relative to center of mass (X [m],
                        // Y [m], Z [m])
  Eigen::Matrix3d inertia;  // 3x3 inertia matrix [kg-m^2]
  auv_core::Matrix62d
    dragCoeffs;    // Drag coefficients (col 0: linear coeffs, col 1: quadratic
                   // coeffs, rows 0-2: translational, rows 3-5: rotational)
  int numThrusters;
  auv_core::Matrix58d thrusterData;  // Pose of thruster relative to CoM
} auvParameters;

/**
 * AUV constraints
 */
typedef struct
{
  double maxXYDistance;        // [m]
  double maxZDistance;         // [m]
  double maxAlignInclination;  // [rad]
  // double closingTolXYZ;      // [m]
  // double closingTolRot;      // [rad]
  Eigen::Vector3d maxTransVel;    // [m/s]
  double maxRotVel;               // [rad/s]
  Eigen::Vector3d maxTransAccel;  // [m/s^2]
  double maxRotAccel;             // [rad/s^2]
  double transJerk;               // [m/s^3]
  // double xyzClosingJerk;     // [m/s^3]
  double rotJerk;  // [rad/s^3]
                   // double rotClosingJerk;     // [rad/^3]
} auvConstraints;
}  // namespace auv_core

#endif  // AUV_STRUCTS_HPP_
