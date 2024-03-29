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
#ifndef ROT3D_HPP_
#define ROT3D_HPP_
#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

// Useful math tools
namespace auv_core
{
namespace rot3d
{
Eigen::Vector4d quat2AngleAxis(
  const Eigen::Quaterniond & quaternion,
  bool nan2Zero = false);

Eigen::Quaterniond angleAxis2Quat(
  const Eigen::Ref<const Eigen::Vector4d> & angleAxis);

Eigen::Quaterniond rpy2Quat(double roll, double pitch, double yaw);

Eigen::Vector3d quat2RPY(const Eigen::Quaterniond & quaternion);

Eigen::Quaterniond relativeQuat(
  const Eigen::Quaterniond & q1,
  const Eigen::Quaterniond & q2);

Eigen::Matrix3d getRotationMat(int axis, double angle);

Eigen::Matrix3d getEulerRotationMat(
  const Eigen::Ref<const Eigen::Vector3d> & rpy);

Eigen::Matrix3d skewSym(const Eigen::Ref<const Eigen::Vector3d> & v);

float sawtoothWave(float x, float period, float max);

float triangularWave(float x, float period, float max);

float mapRollYaw(float x);

float mapPitch(float x);

Eigen::Vector3d getConstrainedRPY(
  const Eigen::Ref<const Eigen::Vector3d> attitude);

Eigen::Vector3d pqr2RPYDot(
  const Eigen::Ref<const Eigen::Vector3d> rpy,
  const Eigen::Ref<const Eigen::Vector3d> pqr);

Eigen::Vector3d rpyDot2PQR(
  const Eigen::Ref<const Eigen::Vector3d> rpy,
  const Eigen::Ref<const Eigen::Vector3d> rpyDot);
}  // namespace rot3d
}  // namespace auv_core

#endif  // ROT3D_HPP_
