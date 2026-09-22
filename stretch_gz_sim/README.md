![](../images/banner.png)

## Overview

*stretch_gz_sim* is an implementation of simulating a Stretch robot with [Gazebo Sim](http://gazebosim.org/) simulator. Compatibility can be found below:

| ROS2 Version | Gazebo Classic | Ignition Fortress | Gazebo Sim Harmonic | Gazebo Sim Jetty |
| -------- | -------| -------| -------            | -------         |
| Foxy     | :x:    | :x:    | :x:                | :x:             |
| humble   | :x:    | :x:    | :x:                | :x:             |
| Jazzy    | :x:    | :x:    | :white_check_mark: | :grey_question: |
| Rolling  | :x:    | :x:    | :grey_question:    | :grey_question: |

- :white_check_mark: - Functional
- :grey_question: - To Be Tested
- :x: - Not Functional

## Running Gazebo
Launching the simulation is as simple as:
```bash
ros2 launch stretch_gz_sim stretch_gz_sim.launch.py
```

## Differential Drive Test
To test the differential drive first install this package:
```bash
sudo apt install ros-<distro>-teleop-twist-keyboard
```
> Note: change `<distro>` for the ros2 version you are using (e.g. ros-jazzy-teleop-twist-keyboard)
Then run the following command to move the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args   -p stamped:=true   -r cmd_vel:=/stretch_diff_drive_controller/cmd_vel
```

## Test joints in the robot
To test the joints in the robot first install this package:
```bash
sudo apt-get install ros-jazzy-rqt-joint-trajectory-controller
```
Then, run the following command to move each joint in the robot:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```