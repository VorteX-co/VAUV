#!/usr/bin/env python3
from pymavlink import mavutil
import time

class px4_getinfo:
    def __init__(self,master):
      self.master=master
      
    
    def getIMU(self):
        while True:
            messageIMU = self.master.recv_match(type='RAW_IMU', blocking=True)
            if msg.get_type() == "RAW_IMU":
                return messageIMU

    def getNav(self):
        while True:
            messageNav = self.master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if msg.get_type() == "NAV_CONTROLLER_OUTPUT":
                return messageNav

    def getServoStatus(self):
        while True:
            messageServo=self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
            if msg.get_type() == "SERVO_OUTPUT_RAW":
                return messageServo
    
    def getRc_channel(self):
        while True:
            messageRc=self.master.recv_match(type='RC_CHANNELS', blocking=True)
            if msg.get_type() == "RC_CHANNELS":
                return messageRc
    def getFlight_modes(self):
        return self.master.mode_mapping().keys()
    
    