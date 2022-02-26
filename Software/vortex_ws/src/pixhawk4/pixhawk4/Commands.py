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

# This class is used to control the AUV
# It contains functionalized pymavlink commands that will control AUV params,
# movements, flight modes and arm disarm commands


class Commands:

    def __init__(self, master):
        self.master = master

    # Method used to change flight mode

    def setFlight_mode(self, mode):
        if mode not in self.master.mode_mapping():
            return False
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        while True:
            # Wait for ACK command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                continue
            break

        return True

    # Method used to Disarm pixhawk4 controller

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)
        return True

    # Method used to Arm pixhawk4 controller

    def arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        return True

    # Generic method for setting RC_Channels
    # RC channels are movement channels that simulate Commands send from a Joystick
    # Receives an array That sets channel values Array
    def set_rc_channel_pwm(self,rc_channel_values):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values)
        return

    # Test thruster Function
    def set_test_motor(self, motor_number, pwm):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0, motor_number,
            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
            pwm, 100, 1, mavutil.mavlink.MOTOR_TEST_ORDER_SEQUENCE, 0)

    def init_channels(self):
        rc_channel_values = [1500,1500,1500,1500,1500,1500,1500,1500]
        self.set_rc_channel_pwm(rc_channel_values)

    def init_servos(self):
        self.set_servo_pwm(1, 1500)
        self.set_servo_pwm(2, 1500)
        self.set_servo_pwm(3, 1500)
        self.set_servo_pwm(4, 1500)
        self.set_servo_pwm(5, 1500)
        self.set_servo_pwm(6, 1500)
        self.set_servo_pwm(7, 1500)
        self.set_servo_pwm(8, 1500)

    def set_servo_pwm(self, servo_n, microseconds):
        """
        Set AUX 'servo_n' output PWM pulse-width.

        Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

        'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
        'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

        """
        # master.set_servo(servo_n+8, microseconds) or:
        self.master.mav.command_long_send(
         self.master.target_system, self.master.target_component,
         mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
         0,            # first transmission of this command
         servo_n + 8,   # servo instance, offset by 8 MAIN outputs
         microseconds,  # PWM pulse-width
         0, 0, 0, 0, 0     # unused parameters
        )
