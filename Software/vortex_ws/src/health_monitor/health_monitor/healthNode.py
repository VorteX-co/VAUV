#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from health_monitor.LeakageSensor import LeakageSensor
import threading

class HealthNode(Node):
    def __init__(self):
        super().__init__("HealthNode")
        leakage_Sensor = LeakageSensor()
        self.dataThread = threading.Thread(
            target=leakage_Sensor.publish_leakage, args=(self,))
        self.publisher = self.create_publisher(#leakage sensor message,
        int,"HealthNode",10
        )
        





def main (args=None):
    rclpy.init(args=args)
    healthNode=Node("HealthNode")
    healthNode.dataThread.start()
    rclpy.spin(healthNode)
    rclpy.shutdown()


if __name__=="__main__":
    main()
