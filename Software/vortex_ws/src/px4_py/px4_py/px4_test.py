#!/usr/bin/env python3
from pymavlink import mavutil
import time
import rclpy
from rclpy.node import Node
from px4_interfaces.msg import SensorStatus
from px4_interfaces.msg import NavController
from px4_interfaces.msg import RcMsg
from px4_interfaces.msg import ServoMsg

from px4_py.px4_getinfo import px4_getinfo
from px4_py.control_px4 import control_px4



class px4_node(Node):

    def __init__(self):
        super().__init__("px4")
        try:
            self.master=mavutil.mavlink_connection("/dev/ttyACM5", baud=115200)
        except:
            self.master=mavutil.mavlink_connection("/dev/ttyACM6", baud=115200)


        #initial disarming for px4
        # master.arducopter_disarm() or:
        
        self.info_px4=px4_getinfo(self.master)
        
        self.control=control_px4(self.master)
        
        control_px4.disarm(self.control)
        
        # self.imu_publisher=self.create_publisher(SensorStatus,"Imu_raw",10)
        # self.nav_publisher=self.create_publisher(SensorStatus,"Nav_raw",10)
        # self.rc_publisher=self.create_publisher(SensorStatus,"Rc_channel",10)
        # self.servo_publisher=self.create_publisher(SensorStatus,"Servo_raw",10)


    def publish_imu(self):
        print('hiiii')
        msg = SensorStatus()
        data=px4_getinfo.getIMU(self.info_px4)
        
        msg.xacc=data.xacc
        msg.yacc=data.yacc
        msg.zacc=data.zacc

        msg.xgyr=data.xgyro
        msg.ygyro=data.ygyro
        msg.zgyro=data.zgyro

        msg.xmag=data.xmag
        msg.ymag=data.ymag
        msg.zmag=data.zmag

        self.imu_publisher.publish(msg)      


    def publish_nav(self):
        msg=NavController
        data=px4_getinfo.getNav(self.info_px4)

        msg.nav_roll=data.nav_roll_
        msg.nav_pitch=data.nav_pitch
        msg.nav_bearing=data.nav_bearing

        msg.target_bearing=data.target_bearing
        msg.wp_dist=data.wp_dist
        msg.alt_error=data.alt_error
        msg.xtrack_error=data.xtrack_error

        self.nav_publisher.publish(msg)
        
    def publish_rc(self):
        msg=RcMsg
        data=px4_getinfo.getRc_channel(self.info_px4)

        msg.chan1_raw=data.chan1_raw
        msg.chan2_raw=data.chan2_raw
        msg.chan3_raw=data.chan3_raw
        msg.chan4_raw=data.chan4_raw
        msg.chan5_raw=data.chan5_raw
        msg.chan6_raw=data.chan6_raw
        msg.chan7_raw=data.chan7_raw
        msg.chan8_raw=data.chan8_raw
        msg.chan9_raw=data.chan9_raw
        msg.chan10_raw=data.chan10_raw
        msg.chan11_raw=data.chan11_raw
        msg.chan12_raw=data.chan12_raw
        msg.chan13_raw=data.chan13_raw
        msg.chan14_raw=data.chan14_raw
        msg.chan15_raw=data.chan15_raw
        msg.chan16_raw=data.chan16_raw
        msg.chan17_raw=data.chan17_raw
        msg.chan18_raw=data.chan18_raw
        

        self.rc_publisher.publish(msg)


    def publish_servo(self):
        msg = ServoMsg
        data=px4_getinfo.getServoStatus(self.info_px4)

        msg.servo1_raw=data.servo1_raw
        msg.servo2_raw=data.servo2_raw
        msg.servo3_raw=data.servo3_raw
        msg.servo4_raw=data.servo4_raw
        msg.servo5_raw=data.servo5_raw
        msg.servo6_raw=data.servo6_raw
        msg.servo7_raw=data.servo7_raw
        msg.servo8_raw=data.servo8_raw
        msg.servo9_raw=data.servo9_raw
        msg.servo10_raw=data.servo10_raw
        msg.servo11_raw=data.servo11_raw
        msg.servo12_raw=data.servo12_raw
        msg.servo13_raw=data.servo13_raw
        msg.servo14_raw=data.servo14_raw
        msg.servo15_raw=data.servo15_raw
        msg.servo16_raw=data.servo16_raw


        self.servo_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    px4 =px4_node()
    while True:
        
        px4_node.publish_imu(px4)
        px4_node.publish_nav(px4)
        px4_node.publish_rc(px4)
        px4_node.publish_servo(px4)
        time.sleep(0.1)
       
    rclpy.spin(px4)    
    rclpy.shutdown()


if __name__== "__main__":
    main()












# Create the connection
# Need to provide the serial port and baudrate
# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

#master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')

# Restart the ArduSub board !
#master.reboot_autopilot()



# Get some information !

# while True:
#     try:
#         print(master.recv_match().to_dict())
#     except:
#         pass
#     time.sleep(0.1)

# master.mav.param_request_read_send(
#     master.target_system, master.target_component,
#     b'PILOT_THR_FILT',
#     -1,
#     mavutil.mavlink.MAV_PARAM_TYPE_REAL32
# )


# messageIMU = master.recv_match(type='RAW_IMU', blocking=True).to_dict()
# messageNav=master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True).to_dict()
# messageServo=master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True).to_dict()
# messageRc=master.recv_match(type='RC_CHANNELS', blocking=True).to_dict()

# print(messageIMU)
# print(messageNav)
# print(messageServo)
# print(messageRc)
