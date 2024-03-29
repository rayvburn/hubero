# hubero

HuBeRo is a framework that simulates typical human behaviour. It was mainly designed for social robotics research purposes.

In most cases researchers conduct their experiments in simulation and then switch to real world studies. For the simulation stage they often implement some teleoperation approaches to provide mobility to human characters involved in studies. These teleoperation methods are not the key element of research so they are not discussed further.

HuBeRo faces the problem, providing **navigation skills** and **realistic animation management** for **simulated human characters**.
Additionally, given that simulator provides realistic 3D model of a person, framework allows more detailed **examination of robot perception** in the simulation.

HuBeRo aims to integrate a simulator (it's actually simulator-agnostic), robotic framework (like ROS) to provide realistic navigation behaviours of people.

Click the thumbnail below for the link with a video presentation of the first revision of the framework (integrated with Gazebo and ROS).

[<img src="https://i.vimeocdn.com/video/864800513-93efca8d48b9eea74b0c118257e833f101ce37b7050501bbfd7121f170868896-d_640">](https://vimeo.com/397552304)

## Overview

HuBeRo architecture assumes that control of a single human (actor) is possible by extension of a simulator plugin. The plugin should provide:
- 3D human representation,
- position control capability,
- human posture animation diversity.

HuBeRo is run from the source code point of view - it is necessary to have an access to inject HuBeRo classes into simulator plugin's source code (extension/customization). The injection allows to create an interface for a specific simulator. The interface allows user to create custom scenarios for his robotic research experiments.

The exemplary HuBeRo application provides interfaces to Gazebo and ROS so developers can incorporate mobile humans into their existing experiments.

Further details on HuBeRo architecture can be found in the [article](https://www.jamris.org/index.php/JAMRIS/article/view/664). Please cite this article if HuBeRo proved to be helpful in your research:

```bibtex
@article{karwowski2021hubero,
title={HuBeRo - a Framework to Simulate Human Behaviour in Robot Research},
volume={15},
url={https://www.jamris.org/index.php/JAMRIS/article/view/664},
DOI={10.14313/JAMRIS/1-2021/4},
number={1},
journal={Journal of Automation, Mobile Robotics and Intelligent Systems},
author={Karwowski, Jarosław and Dudek, Wojciech and Węgierek, Maciej and Winiarski, Tomasz},
year={2021},
month={Jul.},
pages={31-38}
}
```

## Run instructions

### Dependencies

**NOTE**: currently Actors use only LiDAR sensor for navigation, RGBD cameras are turned off so this step can be ommitted.

```bash
sudo apt install ros-$ROS_DISTRO-depthimage-to-laserscan
```

### Clone

Clone with HTTPS/SSH:

```bash
git clone https://github.com/rayvburn/hubero.git
cd hubero
git submodule update --init --recursive
```

### Build

**OPTIONAL**: Workspace preparation (additional flags allow easy debugging with Eclipse IDE):

```bash
catkin config -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_ARG1=-std=c++14 -D__cplusplus=201402L -D__GXX_EXPERIMENTAL_CXX0X__=1
```

#### `ignition`

One should tell catkin which version of `ignition` library is installed. This can be done each time, passing flag for compilation:

```bash
catkin build -DIGN_MATH_VER=<your_version_major>
```

or once, configuring `catkin` (note that this will erase your other flags - so if you have such, append them to the new config):

```bash
catkin config -DIGN_MATH_VER=<your_version_major>
```

Generally, Ubuntu 16 will have `ignition` in version `3`, Ubuntu 18 in version `4` etc. Version can be checked by running:

```bash
ls -l /usr/include/ignition
```

#### Build with `catkin`

Build only HuBeRo packages (remember to configure `IGN_MATH_VER` before):

```bash
catkin build hubero_bringup_gazebo_ros hubero_common hubero_core hubero_gazebo hubero_interfaces hubero_ros hubero_ros_msgs hubero_ros_scenarios
```

## Launch

For run instructions, check `hubero_bringup_gazebo_ros` package.

## Scenarios

For exemplary scenarios, check `hubero_ros_scenarios` package.
