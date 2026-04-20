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

> Note: Documentation is still under development, more to come in the future...

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