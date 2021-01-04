# Vision

## Description
The Vision package is responsible for image enhancement, image processing, and performing multiple deep learning models and computer vision algorithms on the video streams coming from the AUV cameras' to detect its targets to publish the AUV current state to the other modules.

The source code is released under a [GNU GENERAL PUBLIC LICENSE](https://github.com/fatma-mohamed-98/VAUV/blob/master/LICENSE).

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home) <br />
Maintainer: vortex-co, info@vortex-co.com**

The Vision package has been tested under [ROS2](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/)  eloquent on Ubuntu 18.04.
## Table of contents

* [Usage](#usage)
* [Config files](#config-files)
* [Launch files](#launch-files)
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
- Start to run node by node with
~~~
	ros2 run <package_name> <node_name>
~~~
- Or with launch file with
~~~
	ros2 launch <package_name> launch_file_name.launch.py
~~~

## Config files
* image_enhancement_params.yaml
## Launch files
* **...**

## Nodes

### image_enhancement
The image enhancement node is responsible for improving the quality of the raw images before processing. By  applying many filters for histogram equalization, Noise removal, and so on.
#### Subscribed Topics
* **`/raw frame`** ([sensor_msgs/Frame])
  The raw frames from sensor package.
#### Published Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
  The enhanced frames after removing noise and manipulate contrast, darkness, and so on.
  
### object_detection 
Contains deep learning models that take the enhanced frame to get the image details.
#### Subscribed Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
  The enhanced frame that published by image_enhancement node.
#### Published Topics
* **`/image details`** ([str_msgs/Data])
  The image details data contains the target classes and their coordinates.
  

### target_analysis
Contains computer vision algorithms that take the enhanced frames and image details data to get finer details about the detected target.

#### Subscribed Topics
* **`/enhanced frame`** ([sensor_msgs/Frame])
* **`/image details`** ([str_msgs/Data])
#### Published Topics
* **`/object details`** ([str_msgs/Data])
  The object details data contains finer details inside the object itself.
  
### data_fusion
Takes the data from object_detection and target_analysis nodes and handle it to get more informative messages about the AUV current state.

#### Subscribed Topics
* **`/object details`** ([str_msgs/Data])
* **`/image details`** ([str_msgs/Data])
#### Published Topics
* **`/current state`** ([str_msgs/Data])
  The current state data contains informative messages about the AUV current state.

## Hardware
This package acess the following hardware:
* ZED 2 Camera
* Low Light Camera
