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


from pymavlink import mavutil

# This Class is used to get information from pixhawk4
# information as sensors messages
# parameters values
# flight mode


class Getinfo:
    # Constructor of class used to create instance with an
    # attribute master which will hold
    # connection object with pixhawk4
    def __init__(self, master):
        self.master = master

    # Method used to get Scaled_IMU object message

    def getIMU(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1, 1)
        while True:

            messageIMU = self.master.recv_match(
                type='RAW_IMU', blocking=True)

            if messageIMU.get_type() == 'RAW_IMU':
                return messageIMU

    # Method used to get Nav_controller_output object message

    def getNav(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1, 1)
        while True:
            messageNav = self.master.recv_match(
                type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if messageNav.get_type() == 'NAV_CONTROLLER_OUTPUT':
                return messageNav

    # Method used to extract ATTITUDE object Message

    def getAttitude(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1, 1)
        while True:
            messageAttitude = self.master.recv_match(
                type='ATTITUDE', blocking=True)
            if messageAttitude.get_type() == 'ATTITUDE':
                return messageAttitude

    # Method used to get Servo_output_Raw object message

    def getServoStatus(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            1, 1)
        while True:
            messageServo = self.master.recv_match(
                type='SERVO_OUTPUT_RAW', blocking=True)
            if messageServo.get_type() == 'SERVO_OUTPUT_RAW':
                return messageServo

    # Method used to get Rc_channel object message

    def getRc_channel(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            1, 1)
        while True:
            messageRc = self.master.recv_match(
                type='RC_CHANNELS', blocking=True)
            if messageRc.get_type() == 'RC_CHANNELS':
                return messageRc

    def getFlight_modes(self):
        return self.master.mode_mapping().keys()
