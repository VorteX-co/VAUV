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
#include "segment_planner.hpp"

namespace auv_guidance
{
/**
 * @param startPos Starting position
 * @param nominalSpeed Desired cruise speed
 */
SegmentPlanner::SegmentPlanner(
  double distance, double nominalSpeed,
  double accel, int seq)
{
  distance_ = distance;
  if (nominalSpeed <=
    0)
  {
    cruiseSpeed_ = SegmentPlanner::DEFAULT_SPEED;
  } else {
    cruiseSpeed_ = abs(nominalSpeed);
  }

  if (accel == 0 || seq == SegmentPlanner::SEQ_NONE) {   // Default case
    accelerate_ = false;
    acceleration_ = 0;
    accelSeq_ = seq;
  } else {
    if (seq >= SegmentPlanner::SEQ_NONE &&
      seq <= SegmentPlanner::SEQ_BOTH)      // Valid sequence
    {
      accelerate_ = true;
      acceleration_ = accel;
      accelSeq_ = seq;
    } else {
      accelerate_ = false;   // Default to constant speed (accel = 0)
      acceleration_ = 0;
      accelSeq_ = SegmentPlanner::SEQ_NONE;
    }
  }

  // Initialize to zero
  t1_ = 0, t2_ = 0, tMid_ = 0, tEnd_ = 0;
  cruiseDuration_ = 0;
  initialSpeed_ = 0, maxSpeed_ = 0, finalSpeed_ = 0;

  SegmentPlanner::initMotionPlanner();
}

// Initialize Motion Planner - calculate travel parameters
void SegmentPlanner::initMotionPlanner()
{
  // Exit if distance is 0
  if (distance_ == 0) {return;}

  if (!accelerate_) {   // Constant speed
    tEnd_ = distance_ / cruiseSpeed_;
    cruiseDuration_ = tEnd_;
    t2_ = tEnd_;
    initialSpeed_ = cruiseSpeed_;
    maxSpeed_ = cruiseSpeed_;
    finalSpeed_ = cruiseSpeed_;
  } else {   // Will be accelerating for certain portions of travel
    double accelDuration =
      cruiseSpeed_ / acceleration_;    // Assuming accelerating from rest
    double accelDist =
      0.5 * pow(cruiseSpeed_, 2) / acceleration_;    // Or 0.5*acceleration*t^2

    if (accelSeq_ == SegmentPlanner::SEQ_START) {
      if (accelDist <= distance_) {   // Possible: Will be traveling at cruiseSpeed
                                      // at destination
        cruiseDuration_ = (distance_ - accelDist) / cruiseSpeed_;
        t1_ = accelDuration;
        tEnd_ = t1_ + cruiseDuration_;
        t2_ = tEnd_;
        finalSpeed_ = cruiseSpeed_;
      } else {   // Impossible: Will be traveling slower than cruiseSpeed at
                 // destination
        std::stringstream ss;
        ss << "SegmentPlanner: SEQ_START - Will be traveling slower than "
          "cruiseSpeed at destination. Decrease speed or increase "
          "acceleration." <<
          std::endl;
        throw std::runtime_error(ss.str());
      }
    } else if (accelSeq_ == SegmentPlanner::SEQ_END) {
      if (accelDist <= distance_) {   // Possible: Will be able to accelerate from
                                      // cruiseSpeed to rest
        cruiseDuration_ = (distance_ - accelDist) / cruiseSpeed_;
        t1_ = 0;
        t2_ = cruiseDuration_;
        tEnd_ = t2_ + accelDuration;
        initialSpeed_ = cruiseSpeed_;
      } else {   // Impossible: Will have non-zero speed when you reach the
                 // destination
        std::stringstream ss;
        ss << "SegmentPlanner: SEQ_END -  Will have non-zero speed at "
          "destination. Decrease speed or increase acceleration." <<
          std::endl;
        throw std::runtime_error(ss.str());
      }
    } else if (accelSeq_ == SegmentPlanner::SEQ_BOTH) {
      if (2 * accelDist <=
        distance_)     // Will reach cruise speed for some duration >= 0
      {
        cruiseDuration_ = (distance_ - 2 * accelDist) / cruiseSpeed_;
        t1_ = accelDuration;
        t2_ = accelDuration + cruiseDuration_;
        tEnd_ = t2_ + accelDuration;
      } else {   // Will not reach cruiseSpeed during travel
        tMid_ = sqrt(distance_ / acceleration_);
        t1_ = tMid_;
        t2_ = tMid_;
        tEnd_ = tMid_ * 2;
      }
    }
    maxSpeed_ = acceleration_ * t1_;
  }
}

double SegmentPlanner::getTravelTime() {return tEnd_;}

/**
 * @param t Current time for state to be computed (state = [position, speed])
 **/
Vector2d SegmentPlanner::computeState(double t)
{
  Vector2d state;  // [position; speed]
  state.setZero();

  if (!accelerate_) {   // Constant Speed
    if (t >= 0 && t <= tEnd_) {   // Valid time instance
      state(0) = cruiseSpeed_ * t;
      state(1) = cruiseSpeed_;
    } else if (t < 0) {
      state(0) = 0;
      state(1) = cruiseSpeed_;
    } else if (t > tEnd_) {
      state(0) = distance_;
      state(1) = cruiseSpeed_;
    }
  } else if (accelerate_) {
    if (t >= 0 && t <= tEnd_) {   // Valid time instance
      double time1 = 0, time2 = 0, time3 = 0;

      // time1 in [t, t1] range - accelarate from rest to cruiseSpeed
      time1 = (t <= t1_) ? t : t1_;
      state(0) = initialSpeed_ * time1 + 0.5 * acceleration_ * pow(time1, 2);
      state(1) = acceleration_ * time1;

      // time2 in (t1, t2] range - traveling at cruiseSpeed
      time2 = (t > t1_ && t <= t2_) ? (t - t1_) : 0;
      time2 = (t > t2_) ? (t2_ - t1_) : time2;
      if (time2 > 0) {
        state(0) = state(0) + cruiseSpeed_ * time2;   // state(1) remains as is
      }
      // time3 in (t2, tEnd] range - accelerate from cruiseSPeed to rest
      time3 = (t > t2_) ? (t - t2_) : 0;
      if (time3 > 0) {
        state(0) =
          state(0) + maxSpeed_ * time3 - 0.5 * acceleration_ * pow(time3, 2);
        state(1) = maxSpeed_ - acceleration_ * time3;
      }
    } else if (t < 0) {
      state(0) = 0;
      state(1) = initialSpeed_;
    } else if (t > tEnd_) {
      state(0) = distance_;
      state(1) = finalSpeed_;
    }
  }
  return state;
}
}  // namespace auv_guidance
