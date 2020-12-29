# Vision

## Description
Vision is a ros2 package which responsible for mission recognition and object detection. 
* The vision module passes the data to the mission planning module.
* Data contain logs from the deep learning model and image analysis model.
* The logs from the deep learning models contain the detected object class ,and it's coordinates
,and score of detection. 
* The logs of the image analysis model contain image manifest, small object coordinates 
,and parameter values which may vary according to each task.
* Both the deep learning model ,and the image analysis model require a noise-free image.
* So, The input frame (from the camera (Sensor Fusion module)) is passed to a image enhancement node
to manipulates the image to fit the tasks.

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/fatma-mohamed-98/VAUV/blob/master/LICENSE).

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home) <br />
Maintainer: vortex-co, info@vortex-co.com**

The Vision package has been tested under [ROS2] eloquent on Ubuntu 18.04.
## Table of contents
* [Installation](#installation)
* [Usage](#usage)
* [Config files](#config-files)
* [Launch files](#launch-files)
* [Nodes](#nodes)
* [Hardware](#hardware)


## Installation
#### Dependencies
- [Robot Operating System (ROS2)](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) (Install Eloquent as Debian Package).
- [Colcon](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
#### you can  install Dependencies at the first time by the following steps:
* first download **installSystem.sh** and **getRos2.sh** from dependencies folder then run the following commands.
* ##### **System requirements¶**
-   **Target platforms for Eloquent Elusor are [(see REP 2000)](https://www.ros.org/reps/rep-2000.html):**
    - **Tier 1: Ubuntu Linux - Bionic Beaver (18.04) 64-bit**
    - **System setup**
    - **Set locale**
    
           $./installSystem.sh

    - **Get ROS 2 code && Install dependencies using rosdep**
   
           $./getRos2.sh

### Building from Source
#### Building
To build from source, clone the latest version from this repository into your workspace and compile the package using 
~~~
	$cd ~/ros2_ws/VAUV/Software/vortex_ws/src/<package_name>/src/
	$source /opt/ros/eloquent/setup.bash
	$colcon build --symlink-install
~~~

## Usage
First go to project location and source it
~~~
	cd ~/ros2_ws/VAUV/Software/vortex_ws/src/<package_name>/src/
	source install/setup.bash
~~~
- Then we can start node by node with
~~~
	ros2 run <package_name> class_name
~~~
- or with launch file with
~~~
	ros2 launch <package_name> launch_file_name.launch.py
~~~

## Config files
* **...**
## Launch files
* **...**

## Nodes

### image_enhancement
Takes the raw frames and apply filters for image enhancement.
#### Subscribed Topics
* **`/raw frame`** ([sensor_msgs/Frame])
  The raw frames from sensor package.
#### Published Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
  The enhanced frames after removing noise and manipulate contrast,darkness and so on.
  
### object_detection
Takes the enhanced frames as the input to deep learning model (Yolov3/Detectron2/Yolact)
to get the object manifest and verify the output class using a pretrained CNN model.
#### Subscribed Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
  The enhanced frame which published by image_enhancement node.
#### Published Topics
* **`/object manifest`** ([str_msgs/Data])
  The object manifest contain the object class ,and it's coordinates.
  

### image_analysis
Contains computer vision algorithm which take the raw frames 
to get finer details like handle position and direction in dracula task,
closed side coordinates and so on.
#### Subscribed Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
#### Subscribed Topics
* **`/object manifest`** ([str_msgs/Data])
#### Published Topics
* **`/image manifest`** ([str_msgs/Data])
  The image manifest and coordinates of small objects.
  


### Image_Processing_logs
Takes the logs what was detected and where and handle it 
for publish meaningful mesges to other packages.
#### Services
* **`current_state`** ([std_srvs/Trigger])
	Returns information about the current state.
#### Parameters
* **`subscriber_topic`** (string, default: Data)
	The name of the input topic.

[comment]: <> (* **`cache_size`** &#40;int, default: 0&#41;)

[comment]: <> (	The size of the cache.)


## Hardware
This package acess the following hardware:
* ZED Camera
* Low Light Camera


[ROS2]: https://index.ros.org/doc/ros2/
