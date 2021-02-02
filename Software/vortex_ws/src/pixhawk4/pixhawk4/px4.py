#!/usr/bin/env python3

# Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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


# import messages
from custom_ros_interfaces.msg import (NavController, RcMsg, SensorStatus,
                                       ServoMsg)

# import util classes for pixhawk4
from pixhawk4.Getinfo import Getinfo
from pixhawk4.MsgCreate import MsgCreate
from pixhawk4.Setinfo import Setinfo

from pymavlink import mavutil
import rclpy
from rclpy.node import Node
import serial


class px4_node(Node):
    # initilizing node
    def __init__(self):
        super().__init__('px4')
    # auto detection of serial port pixhawk4 connected to unix in
        # serial = mavutil.auto_detect_serial()

        try:
            serial = findSerial()
            # try establishing connection with pixhawk
            self.master = mavutil.mavlink_connection(
                serial[0], baud=115200)
        except serial.SerialException:
            print('Unable to establish connection')
            exit(1)
        self.master.wait_heartbeat()
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)
    # create publishing objects

        self.imu_publisher = self.create_publisher(SensorStatus, 'Imu_raw', 10)
        self.nav_publisher = self.create_publisher(
            NavController, 'Nav_raw', 10)
        self.rc_publisher = self.create_publisher(
            RcMsg, 'Rc_channel', 10)
        self.servo_publisher = self.create_publisher(
            ServoMsg, 'Servo_raw', 10)

    # creating information and control pixhawk objects
        self.info_px4 = Getinfo(self.master)
        self.control = Setinfo(self.master)
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1, 1)
        # self.control.arm()

    # px4 Node publishing functions

    def publish_imu(self):
        data = Getinfo.getIMU(self.info_px4)
        print('hi ')
        message = MsgCreate.getIMUMsg(data)
        print('hi publisher')
        print('xacc:%d', message.xacc)
        self.imu_publisher.publish(message)

    def publish_nav(self):
        data = Getinfo.getNav(self.info_px4)
        message = MsgCreate.getNavMsg(data)
        print('roll:%f', message.nav_roll)
        self.nav_publisher.publish(message)

    def publish_rc(self):
        data = Getinfo.getRc_channel(self.info_px4)
        message = MsgCreate.getRcMsg(data)
        self.rc_publisher.publish(message)

    def publish_servo(self):
        data = Getinfo.getServoStatus(self.info_px4)
        message = MsgCreate.getServoMsg(data)
        self.servo_publisher.publish(message)

# main function that starts px4 node


def main(args=None):
    rclpy.init(args=args)
    px4 = px4_node()
    while True:
        px4_node.publish_imu(px4)
        px4_node.publish_nav(px4)
        px4_node.publish_rc(px4)
        px4_node.publish_servo(px4)

    rclpy.spin(px4)
    rclpy.shutdown()
# function used to detect pixhawk ports its connected to


def findSerial():
    serials = []
    gen_port = '/dev/ttyACM'
    i = 0
    while True:
        if len(serials) == 2:
            return serials
        port = gen_port + '' + i
        i = i+1
        try:
            ser = serial.Serial(port)
            if ser.name == 'Pixhawk 4':
                serials.append(port)
            else:
                continue

        except serial.SerialException:
            continue


if __name__ == '__main__':
    main()
