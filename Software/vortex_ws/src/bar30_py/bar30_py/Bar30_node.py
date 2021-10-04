#!/usr/bin env python3
import rclpy 
from rclpy.node import Node
from bar30_py import ms5837
import threading
from custom_ros_interfaces.msg import Bar30
import time
class Bar30Node(Node):
    def __init__(self):
        super().__init__('Bar30')
        self.bar30= ms5837.MS5837_30BA(8)
        if(not self.bar30.init()):
            self.get_logger().info('Bar30 Failed to initialize')
            return
        self.get_logger().info('Bar30 initialized')
        self.dataThread=threading.Thread(target=self.dataThread_callback,args=())
        self.bar30_publisher=self.create_publisher(Bar30, 'Bar30' ,10)

    def dataThread_callback(self):
        msg=Bar30()
        while(True):
            self.bar30.read()
            msg.pressure=self.bar30.pressure()
            msg.temperature=self.bar30.temperature()
            msg.depth=self.bar30.depth()
            msg.altitude=self.altitude()
            self.bar30_publisher.publish(msg)
            time.sleep(1)
            
        

def main(args=None):
    rclpy.init(args=args)
    Bar30= Bar30Node()
    Bar30.dataThread.start()
    rclpy.spin(Bar30)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
