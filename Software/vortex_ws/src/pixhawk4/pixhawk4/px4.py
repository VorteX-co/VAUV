#!/usr/bin/env python3
import time

import rclpy
# import messages
from custom_ros_interfaces.msg import (NavController, RcMsg, SensorStatus,
                                       ServoMsg)
from pymavlink import mavutil
from rclpy.node import Node

# import util classes for pixhawk4
from pixhawk4.MsgCreate import MsgCreate
from pixhawk4.Getinfo import Getinfo
from pixhawk4.Setinfo import Setinfo


class px4_node(Node):
    # initilizing node
    def __init__(self):
        super().__init__("px4")
    # auto detection of serial port pixhawk4 connected to unix in
        # serial = mavutil.auto_detect_serial()
       
        try:
            # try establishing connection with pixhawk
            self.master = mavutil.mavlink_connection(
                "/dev/ttyACM3", baud=115200)
        except:
            print('Unable to establish connection')
            exit(1)

    # create publishing objects
        self.imu_publisher = self.create_publisher(SensorStatus, "Imu_raw", 10)
        self.nav_publisher = self.create_publisher(NavController, "Nav_raw", 10)
        self.rc_publisher = self.create_publisher(
            RcMsg, "Rc_channel", 10)
        self.servo_publisher = self.create_publisher(
            ServoMsg, "Servo_raw", 10)

    # creating information and control pixhawk objects
        self.info_px4 = Getinfo(self.master)
        self.control = Setinfo(self.master)
        self.control.disarm()

    # px4 Node publishing functions

    def publish_imu(self):
        data = Getinfo.getIMU(self.info_px4)
        message = MsgCreate.getIMUMsg(data)
        print("hi publisher")
        print("xacc:%d",message.xacc)
        self.imu_publisher.publish(message)

    def publish_nav(self):
        data = Getinfo.getNav(self.info_px4)
        message = MsgCreate.getNavMsg(data)
        print("roll:%f",message.nav_roll)
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


if __name__ == "__main__":
    main()
