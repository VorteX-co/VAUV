depth_mpc
======

Nonlinear model predictive control software for the trajectory tracking application of an autonomous underwater vehicle in the vertical plane.
--------
Overview
------

Due to the computational burden of the NMPC for The six degree of freedom motion control, the problem is distributed into two slightly coupled subsystems namely, maneuver system and depth system. This is package is an  implementation of the depth part of the distributed MPC.

--------
Table of contents
------
* [Prerequisites](#Prerequisites)
* [Dependencies](#Dependencies )
* [Parameters](#Parameters)
* [Usage](#Usage)
* [Interface](#Interface)
--------
Prerequisites
------
* ROS2 - Eloquent.

--------
Dependencies
------

* CppAD
* Ipopt 3.12.x
* Eigen3



--------
Parameters
------
| Parameter           |  Description  |
|------------------|--------------------------------------------------------------------------------------------------------------------------------|
| N        | Prediction horizon. |
| dt     | Sampling time. |
| w_ze         | Cost function weight on the heave `z` tracking error. |
| w_psie         | Cost function weight on the `pitch` tracking error. |
| w_we         | Cost function weight on the heave speed `w` tracking error. |
| w_Fz           | Cost function weight on using the heave actuation. |
| w_My           | Cost function weight on using the pitch actuation. |
| w_Fz_dot         | Cost function weight on the rate of using the heave actuation. |
| w_My_dot           | Cost function weight on the rate of using the pitch actuation. |
| Mz       | Total heave mass, rigid body mass + added mass. |
| Mtheta       | Total pitch inertia,rigid body inertia Iyy + added inertia. |
| LDz         | Linear damping in heave. |
| LDtheta         | Linear damping in pitch. |
| QDz         | Quadratic damping in heave. |
| QDtheta         | Quadratic damping in pitch. |

--------
Usage
------

* Launching the depth_mpc node:
  
```sh
  $ ros2 launch depth_mpc depth.launch.py
```

--------
Interface
------

The depth_mpc node publishs and subscribes to the following topics:

- Published topics:
  - **`/rexrov/thruster_manager/input_stamped`** of type `geometry_msgs::msg::WrenchStamped`. The output control forces, namely `Fz` ,  `My`.


- Subscribed topics:
  - **`/guidance/DMPC`** of type `custom_ros_interfaces::msg::DMPC`. Custom ros2 msg contains the required information for the depth MPC to work, namely the `AUV-state`, the `Reference-state` and the `Error-state`.
  
