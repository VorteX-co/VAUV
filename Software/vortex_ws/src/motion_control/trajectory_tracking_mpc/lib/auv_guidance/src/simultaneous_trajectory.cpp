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
#include "simultaneous_trajectory.hpp"

namespace auv_guidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param duration Desired travel duration [s]
 */
SimultaneousTrajectory::SimultaneousTrajectory(
  Waypoint * start, Waypoint * end,
  double duration)
{
  wStart_ = start;
  wEnd_ = end;
  totalDuration_ = duration;

  qRel_.setIdentity();
  xState_.setZero();
  yState_.setZero();
  zState_.setZero();
  angleState_.setZero();
  rotationAxis_.setZero();
  noRotation_ = false;

  SimultaneousTrajectory::initTrajectory();
}

/**
 * \brief Initialize the trajectory
 */
void SimultaneousTrajectory::initTrajectory()
{
  qStart_ = wStart_->quaternion();
  qEnd_ = wEnd_->quaternion();
  qRel_ = qStart_.conjugate() * qEnd_;  // Difference quaternion wrt B-frame (q2
                                        // * q1.conjugate is wrt I-frame)

  Eigen::Vector4d angleAxis = auv_core::rot3d::quat2AngleAxis(qRel_, true);
  if (angleAxis.isApprox(Eigen::Vector4d::Zero())) {noRotation_ = true;}

  angularDistance_ = fabs(auv_core::rot3d::mapRollYaw(angleAxis(0)));
  rotationAxis_ =
    angleAxis
    .tail<3>();        // Get axis relative to Body-frame at starting position
  double angVel = wStart_->angVelB().norm();

  //  std::cout << "ST: start attitude " << std::endl <<
  //  auv_core::rot3d::quat2RPY(qStart_) * 180 / M_PI << std::endl; // Debug
  //  std::cout << "ST: angular distance [deg]: " << angularDistance_ * 180 /
  //  M_PI << std::endl; std::cout << "ST: angular axis, relative to I-frame: "
  //  << std::endl << qStart_.conjugate() * rotationAxis_ << std::endl;
  //  std::cout << "ST: end attitude " << std::endl <<
  //  auv_core::rot3d::quat2RPY(qEnd_) * 180 / M_PI << std::endl; // Debug

  Eigen::Vector3d angleStart = Eigen::Vector3d::Zero();
  Eigen::Vector3d angleEnd = Eigen::Vector3d::Zero();
  angleStart(1) = angVel;
  angleEnd(0) = angularDistance_;

  mjtX_ = new MinJerkTrajectory(wStart_->xI(), wEnd_->xI(), totalDuration_);
  mjtY_ = new MinJerkTrajectory(wStart_->yI(), wEnd_->yI(), totalDuration_);
  mjtZ_ = new MinJerkTrajectory(wStart_->zI(), wEnd_->zI(), totalDuration_);
  mjtAtt_ = new MinJerkTrajectory(angleStart, angleEnd, totalDuration_);
}

/**
 * \brief Return total duration
 */
double SimultaneousTrajectory::getDuration() {return totalDuration_;}

/**
 * @param time Time to compute the state at
 * \brief Computes the trajectory state at the specified time
 */
auv_core::Vector13d SimultaneousTrajectory::computeState(double time)
{
  Vector13d state;
  state.setZero();

  xState_ = mjtX_->computeState(time);
  yState_ = mjtY_->computeState(time);
  zState_ = mjtZ_->computeState(time);
  angleState_ = mjtAtt_->computeState(time);

  // Translational Components
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d uvw = Eigen::Vector3d::Zero();

  // Inertial position expressed in I-frame
  xyz(0) = xState_(0);
  xyz(1) = yState_(0);
  xyz(2) = zState_(0);

  // Inertial velocity expressed in I-frame
  uvw(0) = xState_(1);
  uvw(1) = yState_(1);
  uvw(2) = zState_(1);

  // Rotational Components
  Eigen::Vector3d pqr = Eigen::Vector3d::Zero();
  Eigen::Vector4d quat = Eigen::Vector4d::Zero();

  if (noRotation_) {
    qSlerp_ = qStart_;
  } else if (time >= 0 && time <= totalDuration_) {
    double frac = angleState_(0) / angularDistance_;
    qSlerp_ = qStart_.slerp(frac, qEnd_);
    qSlerp_ = qSlerp_.normalized();
  } else if (time < 0) {
    qSlerp_ = qStart_;
  } else if (time > totalDuration_) {
    qSlerp_ = qEnd_;
  }

  uvw = qSlerp_.conjugate() * uvw;  // Inertial velocity expressed in B-frame

  pqr =
    rotationAxis_ * angleState_(1);    // Angular velocity expressed in B-frame
  quat(0) = qSlerp_.w();
  quat(1) = qSlerp_.x();
  quat(2) = qSlerp_.y();
  quat(3) = qSlerp_.z();

  state.segment<3>(auv_core::constants::STATE_XI) = xyz;
  state.segment<3>(auv_core::constants::STATE_U) = uvw;
  state.segment<4>(auv_core::constants::STATE_Q0) = quat;
  state.segment<3>(auv_core::constants::STATE_P) = pqr;
  return state;
}

/**
 * @param time Time to compute accelerations at
 * \brief Compute the trajectory acceleration at specified time (inertial
 * translational acceleration and time-derivative of angular velocity), both
 * expressed in B-frame.
 */
auv_core::Vector6d SimultaneousTrajectory::computeAccel(double time)
{
  Vector6d accel;
  accel.setZero();

  Eigen::Vector3d inertialTransAccel = Eigen::Vector3d::Zero();
  inertialTransAccel(0) = xState_(2);
  inertialTransAccel(1) = yState_(2);
  inertialTransAccel(2) = zState_(2);
  inertialTransAccel = qSlerp_.conjugate() * inertialTransAccel;

  Eigen::Vector3d pqrDot = Eigen::Vector3d::Zero();
  pqrDot = rotationAxis_ * angleState_(2);

  accel << inertialTransAccel, pqrDot;  // Both expressed in B-frame
  return accel;
}
}  // namespace auv_guidance
