#!/usr/bin/env python3

# This module is used to create messages that will be published by pixhawk
from custom_ros_interfaces.msg import (NavController, RcMsg, SensorStatus,
                                       ServoMsg)

class MsgCreate:
        # This method creates a ros2 message from RAW_IMU object message received from pixhawk4
        def getIMUMsg(data):
            msg = SensorStatus()

            msg.xacc = data.xacc
            msg.yacc = data.yacc
            msg.zacc = data.zacc

            msg.xgyro = data.xgyro
            msg.ygyro = data.ygyro
            msg.zgyro = data.zgyro

            msg.xmag = data.xmag
            msg.ymag = data.ymag
            msg.zmag = data.zmag
            return msg
        # This method creates a ros2 message from NAV_CONTROLLER_OUTPUT object message received from pixhawk4


        def getNavMsg(data):
            msg = NavController()
            msg.nav_roll = data.nav_roll
            msg.nav_pitch = data.nav_pitch
            msg.nav_bearing = data.nav_bearing

            msg.target_bearing = data.target_bearing
            msg.wp_dist = data.wp_dist
            msg.alt_error = data.alt_error
            msg.xtrack_error = data.xtrack_error
            return msg

        # This method creates a ros2 message from RC_CHANNELS object message received from pixhawk4


        def getRcMsg(data):
            msg = RcMsg()
            msg.chan1_raw = data.chan1_raw
            msg.chan2_raw = data.chan2_raw
            msg.chan3_raw = data.chan3_raw
            msg.chan4_raw = data.chan4_raw
            msg.chan5_raw = data.chan5_raw
            msg.chan6_raw = data.chan6_raw
            msg.chan7_raw = data.chan7_raw
            msg.chan8_raw = data.chan8_raw
            msg.chan9_raw = data.chan9_raw
            msg.chan10_raw = data.chan10_raw
            msg.chan11_raw = data.chan11_raw
            msg.chan12_raw = data.chan12_raw
            msg.chan13_raw = data.chan13_raw
            msg.chan14_raw = data.chan14_raw
            msg.chan15_raw = data.chan15_raw
            msg.chan16_raw = data.chan16_raw
            msg.chan17_raw = data.chan17_raw
            msg.chan18_raw = data.chan18_raw

            return msg

        # This method creates a ros2 message from SERVO_OUTPUT_RAW object message received from pixhawk4


        def getServoMsg(data):
            msg = ServoMsg()
            msg.servo1_raw = data.servo1_raw
            msg.servo2_raw = data.servo2_raw
            msg.servo3_raw = data.servo3_raw
            msg.servo4_raw = data.servo4_raw
            msg.servo5_raw = data.servo5_raw
            msg.servo6_raw = data.servo6_raw
            msg.servo7_raw = data.servo7_raw
            msg.servo8_raw = data.servo8_raw
            msg.servo9_raw = data.servo9_raw
            msg.servo10_raw = data.servo10_raw
            msg.servo11_raw = data.servo11_raw
            msg.servo12_raw = data.servo12_raw
            msg.servo13_raw = data.servo13_raw
            msg.servo14_raw = data.servo14_raw
            msg.servo15_raw = data.servo15_raw
            msg.servo16_raw = data.servo16_raw

            return msg
