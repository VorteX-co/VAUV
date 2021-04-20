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


import threading

from custom_ros_interfaces.msg import (Attitude, NavController, RcMsg,
                                       SensorStatus, ServoMsg)
from custom_ros_interfaces.srv import Arm, Heartbeat, PublishData, SetMode

from pixhawk4.Commands import Commands
from pixhawk4.Getinfo import Getinfo
from pixhawk4.Px4_utils import Px4_utils


import rclpy
from rclpy.node import Node


class px4_node(Node):
    # initializing node
    def __init__(self):
        super().__init__('px4')

        self.master = Px4_utils.init_px4()

        self.heartbeat = threading.Thread(
            target=Px4_utils.heart_beats, args=(self,))
        self.publish_data = threading.Thread(
            target=Px4_utils.publish_data, args=(self,))

    # create publishing objects

        self.imu_publisher = self.create_publisher(
            SensorStatus, 'Raw_IMU', 10)
        self.nav_publisher = self.create_publisher(
            NavController, 'Nav_Controller', 10)
        self.rc_publisher = self.create_publisher(
            RcMsg, 'Rc_channel', 10)
        self.servo_publisher = self.create_publisher(
            ServoMsg, 'Servo_raw', 10)

        self.attitude_publisher = self.create_publisher(
            Attitude, 'Attitude', 10)

    # create px4 node services
        self.arm_service = self.create_service(Arm, 'Arm', self.callback_arm)
        self.setmode_service = self.create_service(
            SetMode, 'SetFlightMode', self.callback_setFlightMode)
        self.heartbeat_service = self.create_service(
            Heartbeat, 'StartHeartBeat', self.callback_heartbeat)
        self.data_stream_service = self.create_service(
            PublishData, 'StartPublishData', self.callback_publishdata)

    # creating information and control pixhawk objects

        self.info_px4 = Getinfo(self.master)
        self.control = Commands(self.master)

    # Callback functions of services
    def callback_arm(self, request, response):
        isArm = request.arm
        if isArm:
            response.ack = self.control.arm()
        else:
            response.ack = self.control.disarm()
        return response

    def callback_setFlightMode(self, request, response):
        response.ack = self.control.setFlight_mode(request.mode)
        return response

    def callback_heartbeat(self, request, response):
        if request.start:
            self.heartbeat.start()
            response.ack = True
        return response

    def callback_publishdata(self, request, response):
        if request.start:
            self.publish_data.start()
            response.ack = True
        return response

# main function that starts px4 node


def main(args=None):
    rclpy.init(args=args)
    px4 = px4_node()

    rclpy.spin(px4)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
