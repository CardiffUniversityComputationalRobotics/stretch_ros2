![](../images/banner.png)

## Overview

*stretch_gz_sim* is an implementation of simulating a Stretch robot with [Gazebo Sim](http://gazebosim.org/) simulator. Compatibility can be found below:

| ROS2 Version | Gazebo Classic | Ignition Fortress | Gazebo Sim Harmonic | Gazebo Sim Jetty |
| -------- | -------| -------| -------            | -------         |
| Foxy     | :x:    | :x:    | :x:                | :x:             |
| humble   | :x:    | :x:    | :white_check_mark: | :x:             |
| Jazzy    | :x:    | :x:    | :grey_question:    | :x:             |
| Rolling  | :x:    | :x:    | :grey_question:    | :grey_question: |

- :white_check_mark: - Functional
- :grey_question: - To Be Tested
- :x: - Not Functional

> Note: The package is still under development, more to come in the future...

## Installation
Add this package into your workspace and run the following command in your terminal so that Gazebo Sim can find the meshes:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:~/PATH/TO/WORKSPACE' >> ~/.bashrc
```
> Note: Modify `~/PATH/TO/WORKSPACE` for the path to your workspace, for example: `~/ros2_ws/src`

## Running Gazebo
Launching the simulation is as simple as:
```bash
ros2 launch stretch_gz_sim stretch_gz_sim.launch.py
```

### Teleoperation
To teleoperate the Stretch robot is as simple as running the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Controlling individual joints
The joints which you can control are the following:
- joint_arm_l0
- joint_arm_l1
- joint_arm_l2
- joint_arm_l3
- joint_gripper_finger_left
- joint_gripper_finger_right
- joint_head_pan
- joint_head_tilt
- joint_lift
- joint_wrist_yaw

To control the individual joints within the Stretch robot run the following command:
```bash
gz topic -t "/joint_lift" -m gz.msgs.Double -p "data: 0.3"
```
> Note: Ensure for smooth operation to not pass the limits of the joint position