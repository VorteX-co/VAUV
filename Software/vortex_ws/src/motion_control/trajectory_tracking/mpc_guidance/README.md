mpc_guidance
======

Guidance software desigend for reference generation for our trajectory tracking model predictive controller.

--------
Overview
------

The guidance system consists of the following two sub-systems :
- *Trajectory generator* : Generates 3D time-dependent path from current state to a reference waypoint. We use some work from [auv_gnc](https://github.com/tsender/auv_gnc) for building this module.
- *Steering law* : Generates a smooth heading angle reference for reaching a reference waypoint.

--------
Table of contents
------

* [Dependencies](#Dependencies )
* [Usage](#Usage)
* [Interface](#Interface)
* [ References](#References)

--------
Dependencies
------

* Ceres Solver
* custom_ros_interfaces pkg

--------
Usage
------

* Configure the guidance.yaml parameters file, then launch the node:
  
```sh
  $ ros2 launch mpc_guidance guidance.launch.py
```

* Sending waypoint to the guidance node:

```sh
  $ ros2 topic pub --once /guidance/waypoint geometry_msgs/msg/Point "{x: 68.5 , y: 44.5, z: -4.0}"
```

--------
Interface
------

The guidance node publishs and subscribes to the following topics:

- Published topics:
  - **`/guidance/MMPC`** of type `custom_ros_interfaces/msg/MMPC`. Custom ros2 msg contains the required states by the maneuvering model predicive controller, namely `AUV state`, `Reference state` and `Error state`.
   - **`/guidance/DMPC`** of type `custom_ros_interfaces/msg/DMPC`. Custom ros2 msg contains the required states by the depth model predicive controller, namely `AUV state`, `Reference state` and `Error state`

  
- Subscribed topics:
  - **`/rexrov/pose_gt`** of type `nav_msgs/msg/Odometry`. This is the current state of the vehicle. The odometry msg includes pose and twist information.
  - **`/guidance/point`** of type `geometry_msgs/msg/Point`. This is a desired 3D waypoint.

  

  
 --------
References
------

[1] Handbook of Marine Craft Hydrodynamics and Motion Control, Thor I. Fossen.

