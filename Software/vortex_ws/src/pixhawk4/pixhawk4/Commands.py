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
            print('Unknown mode : {}'.format(mode))
            print('Try: getflight_modes function', )
            return False
        # map mode id in pixhawk4
        mode_id = self.master.mode_mapping()[mode]

        # set new mode id
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            0, mode_id, 0, 0, 0, 0, 0)

        while True:
            # Wait for ACK command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                continue

            print(mavutil.mavlink.enums['MAV_RESULT']
                  [ack_msg['result']].description)
            break

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
    # These methods are used to control AUV movement in 3D space under water :

    # Method sets roll rotation channel
    def set_Roll(self, pwm):
        self.set_rc_channel_pwm(2, pwm)

    # Method sets pitch rotation channel
    def set_pitch(self, pwm):
        self.set_rc_channel_pwm(1, pwm)

    # Method sets yaw rotation channel
    def set_Yaw(self, pwm):
        self.set_rc_channel_pwm(4, pwm)

    # Method controls upwards and downwards movement channel
    def set_Throttle(self, pwm):
        self.set_rc_channel_pwm(3, pwm)

    # Method controls forward and backward movement channel
    def set_Forward(self, pwm):
        self.set_rc_channel_pwm(5, pwm)

    # Method controls Lateral movement channel
    def set_Lateral(self, pwm):
        self.set_rc_channel_pwm(6, pwm)

    # Generic method for setting RC_Channels
    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print('Channel does not exist.')
            return

        rc_channel_values = [65535 for _ in range(9)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channel_values)
