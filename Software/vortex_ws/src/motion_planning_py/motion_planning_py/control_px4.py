#!/usr/bin/env python3
from pymavlink import mavutil
import time

class control_px4:

    def __init__(self,master):
        self.master=master
    
    
    def setFlight_mode(self,mode):
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try: getflight_modes function', )
            return
        mode_id = self.master.mode_mapping()[mode]
# Set new mode
        self.master.mav.command_long_send(
        self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        0, mode_id, 0, 0, 0, 0, 0) 
#    or:
# master.set_mode(mode_id) or:
        # master.mav.set_mode_send(
        # master.target_system,
        # mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        # mode_id)
        while True:
            # Wait for ACK command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                continue
            
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break
    def disarm(self):
        # master.arducopter_disarm() or:
        
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
       
        
       
      
        
    def arm(self):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    def set_Roll(self,pwm):
        set_rc_channel_pwm(2, pwm)
    def set_pitch(self,pwm):
        set_rc_channel_pwm(1, pwm)
    def set_Yaw(self,pwm):
        set_rc_channel_pwm(4, pwm)
    def set_Throttle(self,pwm):
        set_rc_channel_pwm(3, pwm)
    def set_Forward(self,pwm):
        set_rc_channel_pwm(5, pwm)
    def set_Lateral(self,pwm):
        set_rc_channel_pwm(6, pwm)
    