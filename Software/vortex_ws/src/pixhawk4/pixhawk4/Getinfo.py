#!/usr/bin/env python3
import time

from pymavlink import mavutil

# This Class is used to get information from pixhawk4
# information as sensors messages
# parameters values
# flight mode


class Getinfo:
    # Constructor of class used to create instance with an attribute master which will hold connection object with pixhawk4
    def __init__(self, master):
        self.master = master

    # Method used to get Raw_IMU object message
    def getIMU(self):
        while True:
            messageIMU = self.master.recv_match(type='RAW_IMU', blocking=True)
            if messageIMU.get_type() == "RAW_IMU":
                return messageIMU
            else :
                return 

    # Method used to get Nav_controller_output object message
    def getNav(self):
        while True:
            messageNav = self.master.recv_match(
                type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if messageNav.get_type() == "NAV_CONTROLLER_OUTPUT":
                return messageNav
            else :
                return 

    # Method used to get Servo_output_Raw object message
    def getServoStatus(self):
        while True:
            messageServo = self.master.recv_match(
                type='SERVO_OUTPUT_RAW', blocking=True)
            if messageServo.get_type() == "SERVO_OUTPUT_RAW":
                return messageServo
            else :
                return 

    # Method used to get Rc_channel object message
    def getRc_channel(self):
        while True:
            messageRc = self.master.recv_match(
                type='RC_CHANNELS', blocking=True)
            if messageRc.get_type() == "RC_CHANNELS":
                return messageRc
            else :
                return 

    def getFlight_modes(self):
        return self.master.mode_mapping().keys()
