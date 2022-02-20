# hubero

HuBeRo is a framework that simulates typical human behaviour. It was mainly designed for social robotics research purposes.

In most cases researchers conduct their experiments in simulation and then switch to real world studies. For the simulation stage they often implement some teleoperation approaches to provide mobility to human characters involved in studies. These teleoperation methods are not the key element of research so they are not discussed further.

HuBeRo faces the problem, providing **navigation skills** and **realistic animation management** for **simulated human characters**.
Additionally, given that simulator provides realistic 3D model of a person, framework allows more detailed **examination of robot perception** in the simulation.

HuBeRo aims to integrate a simulator (it's actually simulator-agnostic), robotic framework (like ROS) to provide realistic navigation behaviours of people.

## Overview

HuBeRo architecture assumes that control of a single human (actor) is possible by extension of a simulator plugin. The plugin should provide:
- 3D human representation,
- position control capability,
- human posture animation diversity.

HuBeRo is run from the source code point of view - it is necessary to have an access to inject HuBeRo classes into simulator plugin's source code (extension/customization). The injection allows to create an interface for a specific simulator. The interface allows user to create custom scenarios for his robotic research experiments.

The exemplary HuBeRo application provides interfaces to Gazebo and ROS so developers can incorporate mobile humans into their existing experiments.

## Run instructions

### Dependencies

**NOTE**: currently Actors use only LiDAR sensor for navigation, RGBD cameras are turned off so this step can be ommitted.

```bash
sudo apt install ros-$ROS_DISTRO-depthimage-to-laserscan
```

### Build

**OPTIONAL**: Workspace preparation (additional flags allow easy debugging with Eclipse IDE):

```bash
catkin config -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_ARG1=-std=c++14 -D__cplusplus=201402L -D__GXX_EXPERIMENTAL_CXX0X__=1
```

Build HuBeRo packages:

```bash
catkin build hubero_bringup_gazebo_ros hubero_common hubero_core hubero_gazebo hubero_interfaces hubero_ros hubero_ros_msgs hubero_ros_scenarios
```

## Launch

For run instructions, check `hubero_bringup_gazebo_ros` package.

## Scenarios

For exemplary scenarios, check `hubero_ros_scenarios` package.
