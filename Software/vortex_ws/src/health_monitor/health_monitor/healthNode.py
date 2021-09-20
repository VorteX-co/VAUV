#!/usr/bin/env python

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.


import rclpy
from rclpy.node import Node
from custom_ros_interfaces.msg import Health
from health_monitor.LeakageSensor import LeakageSensor
import threading
import time


class HealthNode(Node):
    def __init__(self):
        super().__init__("HealthNode")
        self.leakage_Sensor = LeakageSensor()
        self.dataThread = threading.Thread(
            target=self.dataThread_callback, args=(self, ))
        self.publisher = self.create_publisher(Health, "HealthNode", 10)

    def dataThread_callback(self):
        msg = Health()
        while True:
            msg.leakage = self.leakage_Sensor.takeValues()
            self.publisher.publish(msg)
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    healthNode = HealthNode()
    healthNode.dataThread.start()
    rclpy.spin(healthNode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
