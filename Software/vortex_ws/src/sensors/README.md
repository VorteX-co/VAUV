# Sensors

## Description
The source code is released under [GNU GENERAL PUBLIC LICENSE](https://github.com/fatma-mohamed-98/VAUV/blob/master/LICENSE).

**Author: vortex-co<br />
Affiliation: [VorteX-Co](https://vortex-co.com/home) <br />
Maintainer: vortex-co, info@vortex-co.com**

This package has been tested under [ROS2 Eloquent Elusor](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) on [Ubuntu 18.04](https://releases.ubuntu.com/bionic/).
## Table of contents:
1. [Usage](#usage)
1. [Config files](#config-files)
1. [Launch files](#launch-files)
1. [Nodes](#nodes)
1. [Hardware](#hardware)

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
* **...**
## Launch files
* **...**

## Nodes:
1. [Zed Node](#zed-node)


### Zed Node
This node is referred from [zed-ros2-wrapper @ 11ca648](https://github.com/VorteX-co/zed-ros2-wrapper/tree/11ca6484df8aaf2cc6a0ecb524182c0d93c56c42)
### __Zed Node provides access to the following data:__
___
* Left and right rectified/unrectified images.
* Depth data.
* Colored 3D point cloud.
* IMU data.
* Visual odometry: Position and orientation of the camera.
* Pose tracking: Position and orientation of the camera fixed and fused with IMU data (ZED-M and ZED2 only).
* Detected objects (ZED2 only).
* Persons skeleton (ZED2 only).
### __Installation__:
___
1. [Ubutnu 18.04.](https://releases.ubuntu.com/bionic/)
1. [ZED SDK v3.2 or later.](https://www.stereolabs.com/developers/release/)
1. [CUDA dependency.](https://developer.nvidia.com/cuda-downloads)
1. [ROS2 Eloquent Elusor.](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/)
1. [ZED ROS2 wrapper.](https://github.com/stereolabs/zed-ros2-wrapper)
### __The Zed Node publishes data to the following topics__:
___
Each topic has a common prefix created by the launch file as ~ . Substitute the prefix according to the table below.

| prefix | zed mini camera | zed camera    | zed2 camera    |
|--------|-----------------|---------------|----------------|
| ~      | /zedm/zed_node  | /zed/zed_node | /zed2/zed_node |

### __Left camera__:
* `~/left/camera_info`: Left camera calibration data.
* `~/left/image_rect_color`: Left camera color rectified image.
* `~/left/image_rect_gray`: Left camera gray rectified image.
* `~/left_raw/camera_info`: Left camera raw calibration data.
* `~/left_raw/image_rect_color`: Left camera color unrectified image.
* `~/left_raw/image_rect_gray`: Left camera gray unrectified image.
* `~/rgb/camera_info`: RGB calibration data.
* `~/rgb/image_rect_color`: RGB color rectified image.
* `~/rgb/image_rect_gray`: RGB gray rectified image.
* `~/rgb_raw/camera_info`: RGB raw calibration data.
* `~/rgb_raw/image_rect_color`: RGB color unrectified image.
* `~/rgb_raw/image_rect_gray`: RGB gray unrectified image.

### __Right camera__:
* `~/right/camera_info`: Right camera calibration data.
* `~/right/image_rect_color`: Right camera color rectified image.
* `~/right/image_rect_gray`: Right camera gray rectified image.
* `~/right_raw/camera_info`: Right camera raw calibration data.
* `~/right_raw/image_rect_color`: Right camera color unrectified image.
* `~/right_raw/image_rect_gray`: Right camera gray unrectified image.

### __Stereo pair__:
* `~/stereo/image_rect_color`: side-by-side left/right rectified stereo pair. Calibration data are available in the topics: `~/left/camera_info' and '~/right/camera_info`.
* `~/stereo_raw/image_raw_color`: side-by-side left/right unrectified stereo pair. Calibration data are available in the topics: `~/left_raw/camera_info` and `~/right_raw/camera_info`.

### __Depth and point cloud__:
* `~/depth/camera_info`: Depth camera calibration data.
* `~/depth/depth_registered`: Depth map image registered on left image.
* `~/point_cloud/cloud_registered`: Registered color point cloud.
* `~/confidence/confidence_map`: Confidence image (doubleing point values).
* `~/disparity/disparity_image`: Disparity image.

### __Sensors data__:
* `~/left_cam_imu_transform`: Static transform from left camera to IMU sensor.
* `~/imu/data`: Accelerometer, gyroscope, and orientation data in Earth frame.
* `~/imu/data_raw`: Accelerometer and gyroscope data in Earth frame.
* `~/imu/mag`: Raw magnetometer data (only ZED2).
* `~/atm_press`: atmospheric pressure (only ZED2).
* `~/temperature/imu`: temperature measured by the IMU sensor.
* `~/temperature/left`: temperature measured near the left CMOS sensor.
* `~/temperature/right`: temperature measured near the right CMOS sensor.

### __Positional tracking__:
* `~/pose`: Absolute 3D position and orientation relative to the Map frame (Sensor Fusion algorithm + SLAM).
* `~/pose_with_covariance`: Camera pose referred to Map frame with covariance.
* `~/odom`: Absolute 3D position and orientation relative to the Odometry frame (pure visual odometry for ZED, visual-inertial odometry for ZED-M).
* `~/path_map`: Sequence of camera poses in Map frame.
* `~/path_odom`: Sequence of camera odometry poses in Map frame.

### __Mapping__:
* `~/mapping/fused_cloud`: fused point cloud created when the enable_mapping service is called.

### __Object Detection__:
* `~/obj_det/objects`: object detected frame by frame when the enable_obj_det service is called.

### __The ZED Node provides the following services__:

---
Each service has a common prefix created by the launch file as ~ . Substitute the prefix according to the table below.

| prefix | zed mini camera | zed camera    | zed2 camera    |
|--------|-----------------|---------------|----------------|
| ~      | /zedm/zed_node  | /zed/zed_node | /zed2/zed_node |

* `~/set_pose`: Restarts the Tracking algorithm setting the initial pose of the camera to the value passed as vector parameter &#8594; [X, Y, Z, R, P, Y].
* `~/reset_pos_tracking`: Restarts the Tracking algorithm setting the initial pose to the value available in the param server or to the latest pose set with the service set_pose.
* `~/reset_odometry`: Resets the odometry values eliminating the drift due to the Visual Odometry algorithm. The new odometry value is set to the latest camera pose received from the tracking algorithm.
* `~/start_svo_rec`: Start recording an SVO file. If no filename is provided the default zed.svo is used. If no path is provided with the filename the default recording folder is ~/.ros/.
* `~/stop_svo_rec`: Stop an active SVO recording.
* `~/enable_mapping`: enable spatial mapping.
* `~/enable_obj_det`: enable object detection.
* `~/toggle_svo_pause`: set/reset SVO playing pause. Note: Only available if general.svo_realtime is false and if an SVO as been chosen as input source.

## Hardware
This package uses the following hardware components:
* ZED 2 Camera.

