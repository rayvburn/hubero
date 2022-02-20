# hubero_gazebo

Gazebo simulator interface for HuBeRo - A Framework to Simulate Human Behaviour in Robot Research.

## Overview

HuBeRo is interfaced with Gazebo simulator using custom controller plugin for Gazebo's `ActorPlugin`. This plugin extends generic Gazebo `ModelPlugin` entity.

On notes, how to spawn an Actor in Gazebo world, see `hubero_bringup_gazebo_ros` package description.

Task requesting possibility for user and navigation skills of actor are provided by cooperation with ROS interface, see `hubero_ros` package for details.

Tested with Ubuntu 16.04 and Gazebo 8.6.0.
