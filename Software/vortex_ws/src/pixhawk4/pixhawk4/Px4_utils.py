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

import time

from pixhawk4.Getinfo import Getinfo
from pixhawk4.MsgCreate import MsgCreate

from pymavlink import mavutil

import serial


# This Class contains Basic Methods for px4 Node such as init_px4 to
# establish connection with pixhawk4


class Px4_utils:

    # Method initally establishes connection
    #  with pixhawk and returns to px4 node connection object

    def init_px4():
        serials = Px4_utils.findSerial()
        if serials is not None:
            master = mavutil.mavlink_connection(
                serials[0], baud=115200)
        else:
            print('Unable to establish connection')
            return
        master.wait_heartbeat()
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)
        return master

# Helping Function for init_px4 as it finds
# serial ports pixhawk is connected to and returns a list with serial ports

    def findSerial():

        serials = []
        gen_port = '/dev/ttyACM'
        i = 0
        while i < 20:
            if len(serials) == 2:
                return serials
            port = gen_port + '' + str(i)
            i = i+1
            try:
                ser = serial.Serial(port)

                serials.append(ser.name)

            except serial.SerialException:
                continue

# Method called by Heartbeat thread created in px4 node
# to maintain connection and heartbeats publishing to
# pixhawk4 every 1 sec

    def heart_beats(node):
        node.get_logger().info(
                'HEARTBEAT INITIALIZED')
        while True:
            node.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
            time.sleep(1)

#  Function called by publishing data thread created in px4 node to maintain fetching
#  and publishing data streams every 0.5 seconds

    def publish_data(node):
        while True:
            Px4_utils.publish_imu(node)
            Px4_utils.publish_nav(node)
            Px4_utils.publish_rc(node)
            Px4_utils.publish_servo(node)
            Px4_utils.publish_attitude(node)
            Px4_utils.publish_depth(node)
            

# Following functions uses publisher objects
# created in px4 node to publish data streams from Pixhawk
#
# Methods called to publish data
# it uses Getinfo class to fetch the required data object
# and uses MsgCreate class to get custom message object using fetched data
# then gets publisher instance from px4 node to publish message

    def publish_imu(node):
        data = Getinfo.getIMU(node.info_px4)
        message = MsgCreate.getIMUMsg(data)
        message.header.stamp=node.stamp.now().to_msg()
        node.imu_publisher.publish(message)
        return

    def publish_nav(node):
        data = Getinfo.getNav(node.info_px4)
        message = MsgCreate.getNavMsg(data)
        node.nav_publisher.publish(message)
        return

    def publish_rc(node):
        data = Getinfo.getRc_channel(node.info_px4)
        message = MsgCreate.getRcMsg(data)
        node.rc_publisher.publish(message)
        return

    def publish_servo(node):
        data = Getinfo.getServoStatus(node.info_px4)
        message = MsgCreate.getServoMsg(data)
        node.servo_publisher.publish(message)
        return

    def publish_attitude(node):
        data = Getinfo.getAttitude(node.info_px4)
        message = MsgCreate.getAttitudeMsg(data)
        node.attitude_publisher.publish(message)
        return

    def publish_depth(node):
        data = Getinfo.get_depth(node.info_px4)
        message = MsgCreate.getDepthMsg(data)
        node.depth_publisher.publish(message)
        return

    #Maps service Message to Rc channel list sent to set_rc_channel
    def set_thrusters(self, message):
        #Rc channel list Structure [Pitch PWM, ROLL PWM, THROTTLE PWM, YAW PWM, FORWARD PWM, LATERAL PWM, CAMERA PAN PWM (SET TO DEFAULT 1500 WHILE MOVING), CAMERA TILT PWM (SET TO DEFAULT 1500 WHILE MOVING)]
        rc_channel_values = [message.pitch,message.roll,message.throttle,message.yaw,message.forward,message.lateral,1500,1500]
        self.control.set_rc_channel_pwm(rc_channel_values)
        return True


    def set_LedLights(self, message):
        if message.servo > 8 or message.servo < 1:
            return False
        if message.pwm > 1900 or message.pwm < 1100:
            return False
        self.control.set_servo_pwm(message.servo_n, message.pwm)
        return True
