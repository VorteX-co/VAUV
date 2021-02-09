# Sensors interfacing Package

## Description

The sensors packages is a ros2 package that encapsulates sensor drivers and sensor fusion nodes for the VAUV's onboard sensors.

### License

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/VorteX-co/VAUV/blob/master/LICENSE)

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**


## Table of contents

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [DVL Sensor driver](#DVL-Sensor)
  * [A50 DVL driver Description](#A50-DVL-Description)
  * [A50 DVL driver Usage](#A50-DVL-Usage)
  * [A50 DVL driver Nodes](#A50-DVL-Nodes)


## Prerequisites

* ROS2 - Eloquent. 

## Dependencies 

* ros_custom_interfaces package 
* Boost.Asio

## DVL-Sensor

## A50-DVL-Description

The DVL node is a ros2 driver  for interfacing with the serial port connected to the I/O module of the waterlinked A50-DVL. 

## A50-DVL-Usage

1- Build the sensors/custom_ros_interfaces packages
```
$ cd ~/vortex_ws
$ colcon build --packages-select custom_ros_interfaces sensors
```
2- In a new terminal, source the workspace and run the dvl node 
```
$ cd ~/vortex_ws
$ . install/setup.bash
$ ros2 run sensors pub  // the default port name is "/dev/ttyUSB1"
```
* Or run the dvl node with the port name as an argument
```
$ ros2 run sensors dvl --ros-args -p Port:="/dev/ttyUSB2"  // change 2 to the desired number
```
* To check the USB port name connect to the waterlinked DVL run the following command:
```
$ python -m serial.tools.miniterm
```

## A50-DVL-Nodes

### The Serial Node

* Topic name = "dvl_report"
* Message name = "DVL.msg"
* DVL.msg components:
  * time: Milliseconds since last velocity report (ms)
  * Vx: Measured velocity in x direction (m/s)
  * Vy: Measured velocity in y direction (m/s)
  * Vz: Measured velocity in z direction (m/s)
  * fom: Figure of merit, a measure of the accuracy of the measured velocities (m/s)
  * altitude: Measured altitude to the bottom (m)
  * valid: If valid is "y" the DVL has lock on the bottom and the altitude and velocities are valid (y/n)
  * status: 0 for normal operation, 1 for high temperature warning







