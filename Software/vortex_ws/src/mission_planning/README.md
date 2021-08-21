# Mission Planning Package

## Description

It starts with the **planning layer** which contains **task manager**, it sends next mission to be executed to the execution layer based on the labeled camera feed and mission priority.
In the **execution layer** :
* The **task controller** takes input (from task manager) the next mission to be executed and the task feedback ( from task failure handler which takes its readings from sensor fusion), then splits the mission to primitive tasks in correct order to stack them in the scheduler.

* The **scheduler**  sends tasks id to **global planner** to start execution and also **task scheduler** sends task id to **task failure handler** for monitoring the mission execution.

* The **feedback handler**  takes input (from scheduler) the current executing mission ID to help knowing what feedback threshold to expect and which sensors to get its readings from, then sends feedback on each task to task controller and the **task controller** provide a general feedback from several tasks connected to the same mission and send this general feedback to task manager to be able to log the status in log file ,here we have two cases 
    * In case of success therefore there is nothing to be done!	 
    * In case of failure, the task manager decides to redo the mission with a number of attempts and it may also delete the mission and its dependent mission from the scheduler.


### License

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/fatma-mohamed-98/VAUV/blob/master/LICENSE).

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home)<br />
Maintainer: vortex-co, info@vortex-co.com**

The mission planning module package has been tested under [ROS2](https://index.ros.org/doc/ros2/) Eloquent Elusor on Ubuntu 18.04.

## Table of contents
* [Usage](#Usage)
* [Config files](#Config-files)
* [Launch files](#Launch-files)
* [Nodes](#Nodes)
* [Hardware](#hardware)


## Usage

- Source your ROS2 installation
~~~
	source /opt/ros/eloquent/setup.bash
~~~
- Navigate to your workspace
~~~
	cd ~/$HOME/VAUV/Software/vortex_ws
~~~
- Build your package 
~~~
	colcon build --packages-select mission_planning
~~~
- Start to run node by node with
~~~
	ros2 run mission_planning <node_name>
~~~
- Or with launch file with
~~~
	ros2 launch mission_planning launch_file_name.launch.py
~~~

## Config files

...

## Launch files

...

## Nodes

### task_manager node

Reads labeled camera feed ,detects the mission the AUV is currently seeing and determines the next mission to be executed.

#### Subscribed Topics

...

#### Published Topics

...

#### Services

...

#### Parameters

...

### task_controller node

Maps each mission id to a corresponding finite state machine, Loops on each current state in the FSM until this state changes based on the feedback from the task failure handler until the FSM is finished and another one takes its place.

#### Subscribed Topics

...

#### Published Topics

...

#### Services

...

#### Parameters

...

### scheduler node

Takes task ID to schedule it, manages the available time for a series of tasks and determines which scheduled task to be executed.


#### Subscribed Topics

...

#### Published Topics

...

#### Services

...

#### Parameters

...

### task_failure_handler node

Takes the current executing task ID and Subscribes on sensor’s topics to get its readings, then publishes feedback on each task.

#### Subscribed Topics	

...

#### Published Topics
	
...

#### Services

...

#### Parameters

...

## Hardware

This package is only for planning, So it doesn't directly use any sort of hardware.