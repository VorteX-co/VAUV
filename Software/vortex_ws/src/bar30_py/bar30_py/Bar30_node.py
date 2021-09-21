#!/usr/bin env python3
import rclpy 
from rclpy.node import Node
import ms5837
import threading


class Bar30Node(Node):
    def __init__(self):
        super().__init__('Bar30')
        self.get_logger().info('Bar30 initialized')
        self.bar30= ms5837()
        self.bar30.init()
        self.dataThread=threading.Thread()
        self.bar30_publisher=self.create_publisher(#message type int 
        int, 'Bar30' ,10)

    def dataThread_callback(self):
        while(True):
            self.bar30.read()
            
        

def main(args=None):
    rclpy.init(args=args)
    Bar30= Bar30Node()
    rclpy.spin(Bar30)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
