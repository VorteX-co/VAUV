# Sensors interfacing Package

## Description

This package contains the sensors low-level hardware drivers of Vortex AUV wrapped by ROS2 C++ nodes.

### License

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/VorteX-co/VAUV/blob/master/LICENSE)

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**

## Table of contents

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [DVL Sensor](#DVL-Sensor)
          * [ Description](#Description)
          * [ Usage](#Usage)
          * [Info](#Info)

## Prerequisites

* ROS2 - Eloquent. 

## Dependencies 

* ros_custom_interfaces package 

## DVL-Sensor

## Description

* The sensors packages contains a c++ node for serial-interfacing with waterlinked A50 DVL. Before executing the dvl_serial.cpp node you should establish the following connection:
 
## Usage

1- Build the sensors packages
```
$ cd ~/vortex_ws
$ colcon build --packages-select sensors
```
2- In a new terminal, source the workspace and run the dvl node 
```
$ cd ~/vortex_ws
$ . install/setup.bash
$ ros2 run sensors pub // the default port name is "/dev/ttyUSB0"
```
3- Passing the port name to the dvl node as an argument
```
$ cd ~/vortex_ws
$ . install/setup.bash
$ ros2 run sensors  pub --ros-args -p  Port:="/dev/ttyUSB0"
```

## Info

### DVL Serial Node interface

  * Topic name = "Vortex/DVL"
  * Message name = "DVL.msg"

  Where DVL.msg is a custom msg has the following components:
  * stamp: current rclcpp::clock stamp (s)
  * dt: time passed since last velocity report (s)
  * twist: vector of Measured velocity in [x,y,z] directions (m/s)
  * variance: estimated value based on measured figure of merit (m/s)^2
  * translation: vector of estimated translations in [x_dvl, y_dvl, z_dvl] due to the measured velocity (m)
  * altitude: Measured altitude to the bottom (m)
  * valid: if ture then the DVL has lock on the bottom and the altitude and velocities are valid (true/false)
  * status: if false  then the DVL in normal operation, true then there is a high temperature warning (true/false)
  * count: count of  velocity reports from initializing the conection with the serial port
