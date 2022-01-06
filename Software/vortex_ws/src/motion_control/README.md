--------
Motion Control
------

Motion control system ROS2 package for autonomous underwater vehicles.

--------
Table of contents
------

* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [Nodes](#Nodes)
* [Usage](#Usage)
* [Documentation](#Documentation)

--------
Prerequisites
------

* ROS2 - Eloquent.

--------
Dependencies
------

* [ruckig](https://github.com/pantor/ruckig)
* Eigen >= 3.4
* CppAD
* Ipopt 3.12.x
* custom_ros_interfaces package

* Use this [script](https://github.com/VorteX-co/VAUV/tree/master/tools/scripts/mpc_setup) for dependencies installation.

--------
Nodes
------

### controller_node

- Subscribed topics:
  - **`/odometry/filtered`** of type `nav_msgs/msg/Odometry`. The current state of the vehicle [pose η, velocity ν].
  - **`/Controller/cmd_waypoint`** of type `geometry_msgs/msg/Point`. A desired 3D waypoint.
  - **`/Controller/cmd_attitude`** of type `geometry_msgs/msg/Point`. A desired 3D attitude [rpy].

- Published topics:
  - **`/swift/thruster_manager/input_stamped`** of type `geometry_msgs::msg::WrenchStamped`. The output control forces and moments.

--------
Usage
------

* Adapt the parameters file config.yaml according to your vehicle.
* Launch the controller

```sh
  $ ros2 launch motion_control mpc.launch.py
```

* Sending a reference waypoint in ENU inertial frame:

```sh
  $ ros2 topic pub --once /Controller/cmd_waypoint geometry_msgs/msg/Point "{x: 10.0 , y: 10.0, z: -10.0}"
```

--------
Documentation
------

* [Modelling](docs/model/model.md)
* [Guidance](docs/guidance/guidance.md)
* [Control](docs/control/mpc.md)

