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
#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

// Useful math tools
namespace auv_core
{
namespace constants
{
// Mathematical Constants and Properties
static constexpr double GRAVITY = 9.80665;     // [m/s^2]
static constexpr double WATER_DENSITY = 1000;  // [kg/m^3]

// Full state vector indeces
static const int STATE_XI = 0;  // Inertial X-pos, expressed in I-frame
static const int STATE_YI = 1;  // Inertial Y-pos, expressed in I-frame
static const int STATE_ZI = 2;  // Inertial Z-pos, expressed in I-frame
static const int STATE_U = 3;   // Inertial X velocity , expressed in B-frame
static const int STATE_V = 4;   // Inertial Y velocity , expressed in B-frame
static const int STATE_W = 5;   // Inertial Z velocity , expressed in B-frame
static const int STATE_Q0 = 6;  // Quaternion (I->B Frame) scalar
static const int STATE_Q1 = 7;  // Quaternion (I->B Frame) i-component
static const int STATE_Q2 = 8;  // Quaternion (I->B Frame) j-component
static const int STATE_Q3 = 9;  // Quaternion (I->B Frame) k-component
static const int STATE_P =
  10;    // Inertial X angular velocity , expressed in B-frame
static const int STATE_Q =
  11;    // Inertial Y angular velocity , expressed in B-frame
static const int STATE_R =
  12;    // Inertial Z angular velocity , expressed in B-frame

// Reduced state vector indeces (only Q0 is omited)
static const int RSTATE_XI = 0;  // Inertial X-pos, expressed in I-frame
static const int RSTATE_YI = 1;  // Inertial Y-pos, expressed in I-frame
static const int RSTATE_ZI = 2;  // Inertial Z-pos, expressed in I-frame
static const int RSTATE_U = 3;   // Inertial X velocity , expressed in B-frame
static const int RSTATE_V = 4;   // Inertial Y velocity , expressed in B-frame
static const int RSTATE_W = 5;   // Inertial Z velocity , expressed in B-frame
static const int RSTATE_Q1 = 6;  // Quaternion (I->B Frame) i-component
static const int RSTATE_Q2 = 7;  // Quaternion (I->B Frame) j-component
static const int RSTATE_Q3 = 8;  // Quaternion (I->B Frame) k-component
static const int RSTATE_P =
  9;    // Inertial X angular velocity , expressed in B-frame
static const int RSTATE_Q =
  10;    // Inertial Y angular velocity , expressed in B-frame
static const int RSTATE_R =
  11;    // Inertial Z angular velocity , expressed in B-frame
static const int RSTATE_XI_INT = 12;  // Integrator for STATE_XI
static const int RSTATE_YI_INT = 13;  // Integrator for STATE_YI
static const int RSTATE_ZI_INT = 14;  // Integrator for STATE_ZI
static const int RSTATE_Q1_INT = 15;  // Integrator for STATE_Q1
static const int RSTATE_Q2_INT = 16;  // Integrator for STATE_Q2
static const int RSTATE_Q3_INT = 17;  // Integrator for STATE_Q3
}  // namespace constants
}  // namespace auv_core

#endif   // CONSTANTS_HPP_
