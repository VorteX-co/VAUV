--------
Overview
------

Linear Quadratic Regulator for 6DOF AUV tracking control problem.

--------
Table of contents
------

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [Nodes](#Nodes)
* [Usage](#Usage)
* [Documentation](#Documentation)
* [References](#References)

--------
Prerequisites
------

* ROS2 - Eloquent.

--------
Dependencies
------

* [ruckig](https://github.com/pantor/ruckig)
* Eigen3
* [lin_alg_tools](https://github.com/jerelbn/lin_alg_tools)
* custom_ros_interfaces package

--------
Nodes
------

### controller_node

- Subscribed topics:
  - **`/odometry/filtered`** of type `nav_msgs/msg/Odometry`. The current state of the vehicle [pose η, velocity ν].
  - **`/LQR/cmd_waypoint`** of type `geometry_msgs/msg/Point`. A desired 3D waypoint.
  - **`/LQR/cmd_roll`** of type `std_msgs/msg/Float32`. A desired roll-angle.
  - **`/LQR/cmd_pitch`** of type `std_msgs/msg/Float32`. A desired pitch-angle.
  - **`/LQR/cmd_yaw`** of type `std_msgs/msg/Float32`. A desired yaw-angle.
  - **`/LQR/cmd_hold`** of type `std_msgs/msg/Float32`. A station keeping command.

- Published topics:
  - **`/swift/thruster_manager/input_stamped`** of type `geometry_msgs::msg::WrenchStamped`. The output control forces and moments.

- Service Clients:
 - **`control_pwm`** of type `custom_ros_interfaces/srv/PWM`. Output control PWM  to the autopilot Pixhawk4.

--------
Usage
------

* Launching the node:

```sh
  $ ros2 launch trajectory_tracking_lqr lqr.launch.py
```

* Sending a reference waypoint in ENU inertial frame:

```sh
  $ ros2 topic pub --once /LQR/cmd_waypoint geometry_msgs/msg/Point "{x: 48.5 , y: 35.5, z: -40.0}"
```

* Sending a reference roll-angle in ENU inertial frame:

```sh
  $ ros2 topic pub --once /LQR/cmd_roll std_msgs/msg/Float32 "{data: 0.5}"
```

* Sending a reference pitch-angle in ENU inertial frame:

```sh
  $ ros2 topic pub --once /LQR/cmd_pitch std_msgs/msg/Float32 "{data: 0.7071}"
```

* Sending a reference yaw-angle in ENU inertial frame:

```sh
  $ ros2 topic pub --once /LQR/cmd_yaw std_msgs/msg/Float32 "{data: 0.52}"
```

--------
Documentation
------

The LQR computes a control law for a given system such that a certain optimality criterion is achieved. This is usually a cost function that depends on the state and control variables. Consider the linear state space model `ẋ = Ax + Bu` The feedback control law for the system is
is computed as `τ = -K (x-xd)` where K is the optimal control gain founded by solving the Algebraic Riccati equation, for this purpose we are using the [lin_alg_tools](https://github.com/jerelbn/lin_alg_tools) library which contains an efficient solution to the (ARE).


 --------
References
------

[1] Handbook of Marine Craft Hydrodynamics and Motion Control, Thor I. Fossen.

