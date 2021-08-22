#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import LeakageSensor

class HealthNode(Node):
    def __init__(self):
        super().__init__("HealthNode")
        





def main (args=None):
    rclpy.init(args=args)
    healthNode=Node("HealthNode")
    rclpy.spin(healthNode)
    rclpy.shutdown()


if __name__=="__main__":
    main()
