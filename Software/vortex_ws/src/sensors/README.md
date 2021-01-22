# Sensors interfacing Package

## Description

Sensors is a ros2 package encapsulates c++ nodes required for establishing a low-level interfacing with the AUV's onboard-sensors.

### License

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/VorteX-co/VAUV/blob/master/LICENSE)

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**


## Table of contents

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [DVL Sensor](#DVL-Sensor)
  * [A50 DVL Description](#A50-DVL-Description)
  * [A50 DVL Usage](#A50-DVL-Usage)
  * [A50 DVL Nodes](#A50-DVL-Nodes)


## Prerequisites

* ROS2 - Eloquent. 

## Dependencies 

* ros_custom_interfaces package 

## DVL-Sensor

## A50-DVL-Description

* The sensors packages contains a c++ node for serial-interfacing with waterlinked A50 DVL. Before executing the dvl_serial.cpp node you should establish the following connection:
 
* ![wiring_dvl](./img/wiring_dvl.png)

## A50-DVL-Usage

1- Build the sensors packages
```
$ cd ~/vortex_ws
$ colcon build --packages-select sensors
```
2- In a new terminal, source the workspace and run the dvl node 
```
$ cd ~/vortex_ws
$ . install/setup.bash
$ ros2 run sensors dvl  // the default port name is "/dev/ttyUSB1"
```
* Or run the dvl node with the port name as an argument
```
$ ros2 run sensors dvl --ros-args -p Port:="/dev/ttyUSB2"  // change 2 to the desired number
```
* To check the USB port name connect to the waterlinked DVL run the following command:
```
$ python -m serial.tools.miniterm
```
* To give an administrative permission to the respective USB port:
```
$ sudo chmod 666 /dev/ttyUSB2
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






