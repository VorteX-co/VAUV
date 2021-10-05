#!/usr/bin/env python3

# Copyright 2020-2021 Vortex-co.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from custom_ros_interfaces.msg import (Attitude, Depth, NavController, RcMsg,
                                       SensorStatus, ServoMsg)
from sensor_msgs.msg import Imu

# This module is used to create messages that will be published by pixhawk


class MsgCreate:
    # This method creates a ros2 message
    # from RAW_IMU object message received from pixhawk4

    def getIMUMsg(data):
        msg = Imu()
        msg.header.frame_id='swift/imu_link'
    
        msg.linear_acceleration.x = data.xacc /1000 # G
        msg.linear_acceleration.y = data.yacc /1000 # G
        msg.linear_acceleration.z = data.zacc /1000 # G
        msg.linear_acceleration_covariance [0] = 0.00075
        msg.linear_acceleration_covariance [4] = 0.00075
        msg.linear_acceleration_covariance [8] = 0.00075

        msg.angular_velocity.x = data.xgyro  /1000 # mrad/s
        msg.angular_velocity.y = data.ygyro  /1000 # mrad/s
        msg.angular_velocity.z = data.zgyro  /1000 # mrad/s

        msg.angular_velocity_covariance[0] = 0.00122173
        msg.angular_velocity_covariance[4] = 0.00122173
        msg.angular_velocity_covariance[8] = 0.00122173


        # msg.xmag = data.xmag  # mgauss
        # msg.ymag = data.ymag  # mgauss
        # msg.zmag = data.zmag  # mgauss
        return msg
    # This method creates a ros2 message
    # from NAV_CONTROLLER_OUTPUT object message received from pixhawk4

    def getNavMsg(data):
        msg = NavController()
        msg.nav_roll = data.nav_roll  # current desired roll                   deg
        msg.nav_pitch = data.nav_pitch  # current desired pitch                  deg
        msg.nav_bearing = data.nav_bearing  # current desired heading                deg
        # Bearing to current target              deg
        msg.target_bearing = data.target_bearing
        msg.wp_dist = data.wp_dist  # Distance to active waypoint            meters
        msg.alt_error = data.alt_error  # Current altitude error                 meters
        msg.xtrack_error = data.xtrack_error  # Current crosstrack error on x-y plane  m
        return msg

    # This method creates a ros2 message
    # from RC_CHANNELS object message received from pixhawk4

    def getRcMsg(data):  # ALL IN microSeconds
        msg = RcMsg()
        msg.chan1_raw = data.chan1_raw
        msg.chan2_raw = data.chan2_raw
        msg.chan3_raw = data.chan3_raw
        msg.chan4_raw = data.chan4_raw
        msg.chan5_raw = data.chan5_raw
        msg.chan6_raw = data.chan6_raw
        msg.chan7_raw = data.chan7_raw
        msg.chan8_raw = data.chan8_raw
        return msg

    # This method creates a ros2 message
    # from SERVO_OUTPUT_RAW object message received from pixhawk4

    def getServoMsg(data):  # ALL in microseconds
        msg = ServoMsg()
        msg.servo1_raw = data.servo1_raw
        msg.servo2_raw = data.servo2_raw
        msg.servo3_raw = data.servo3_raw
        msg.servo4_raw = data.servo4_raw
        msg.servo5_raw = data.servo5_raw
        msg.servo6_raw = data.servo6_raw
        msg.servo7_raw = data.servo7_raw
        msg.servo8_raw = data.servo8_raw
        return msg

    def getAttitudeMsg(data):
        msg = Attitude()
        msg.roll = data.roll  # radians
        msg.rollspeed = data.rollspeed  # radians/s
        msg.pitch = data.pitch  # radians
        msg.pitchspeed = data.pitchspeed  # radians/s
        msg.yaw = data.yaw  # radians
        msg.yawspeed = data.yawspeed  # radians/s
        return msg

    def getDepthMsg(data):
        msg = Depth()
        msg.depth = data.press_abs  # meters
        return msg
