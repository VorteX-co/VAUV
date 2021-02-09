# Vision


## Description

-The Pixhawk4 module is responsible establishing connection with pixhawk, retreiving sensors data from pixhawk, send command signals to pixhawk to set parameters or to control Actuators and thrusters using mavlink protocol



The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/fatma-mohamed-98/VAUV/blob/master/LICENSE).

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home) <br />
Maintainer: vortex-co, info@vortex-co.com**

The Vision package has been tested under [ROS2](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/)  eloquent on Ubuntu 18.04.

## Table of contents
 * [Usage](#usage) 
 * [Config files](#config-files) 
 * [Nodes](#nodes) 
 * [Hardware](#hardware)

## Usage

- Source your ROS2 installation.
~~~
source /opt/ros/eloquent/setup.bash
~~~
- Navigate to your workspace
~~~
cd ~/$HOME/VAUV/Software/vortex_ws
~~~
- Build your package
~~~
colcon build --packages-select <package_name>
~~~

##Config files
**...**

## Launch files 
**...**

## Nodes

### px4
px4 is the node responsible for establishing connections with pixhawk4 and retreive data from it, it's also responsible for receiving commands from control module to send it as mavlink messages to pixhawk, the node is the main interface of pixhawk with rest of the AUV
#### Subscribed Topics
**...**
#### Published Topics
**'/Nav_controller data'** ([custom_ros_interfaces/Msg/Nav_controller])
**'/Attitude data'** ([custom_ros_interfaces/Msg/Attitude])
**'/Rc_channel data'** ([custom_ros_interfaces/Msg/Rc_channel])
**'/Scaled_IMU data'** ([custom_ros_interfaces/Msg/Scaled_IMU])
**'/Servo_raw data'** ([custom_ros_interfaces/Msg/Servo_raw])

## Hardware 
Pixhawk4 packages accesses the following hardware:
* Pixhawk 4



