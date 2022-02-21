# hubero_ros
ROS interface for A Framework to Simulate Human Behaviour in Robot Research

`hubero_ros` provides navigation skills to actors (paths planned with ROS Navigation stack algorithms). This package also allows requesting tasks from actors using ROS interface.

## Main features

How does this package cooperate with HuBeRo core?
- each Actor in HuBeRo is initialized with `NavigationRos` (which extends `NavigationBase`)
- each Actor in HuBeRo is initialized with `TaskRequestRos` (which extends `TaskRequestBase`)

How is this package connected to ROS Navigation stack?
- `NavigationRos` class connects to `move_base` Action Server - this allows to pass navigation commands to each actor.

Actor TFs are published to ROS via `NavigationRos` plugin. To investigate it further, run an exemplary world and check ROS TF tree:

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

Actor task requests and execution are realized using ROS Action Server/Client architecture.

## Nodes

Since `ActorPlugin` is "spawned" as Gazebo plugin, nodes are not separated, but most of the data published to ROS topics come from `/gazebo` publisher.

## Topics

General rule is as follows:

- actor resources are placed in: `/hubero/<ACTOR_NAME>/` namespace
- `move_base` resources are placed in: `/hubero/<ACTOR_NAME>/navigation/`
- sensor data are placed in: `/hubero/<ACTOR_NAME>/receptor/` namespace
- actor status message is published to: `/hubero/<ACTOR_NAME>/status` topic
- actor task-related topics are placed in: `/hubero/<ACTOR_NAME>/task/<TASK_NAME>/` namespace (action topics involve `<goal,feedback,result>`)

## Actions

Actions can be **started** without any scenario. One may use topic publisher tool and apply a pattern of:

```bash
rostopic pub --once /hubero/<ACTOR_NAME>/task/<TASK_NAME>/goal
```

then `TAB-TAB` and fill up the action goal structure.

Actions can be **cancelled** the same way using:

```
rostopic pub --once /hubero/<ACTOR_NAME>/task/<TASK_NAME>/cancel
```

then `TAB-TAB` and fill up the action goal structure.

There is also a convenient GUI tool for manual commanding: [`actionlib axclient.py`](https://answers.ros.org/question/10845/command-line-action-server-interface/?answer=16022#post-id-16022)

## Parameters

Configurable parameters are settable via `actor.launch` file. See the file for a detailed description. For example how to override default parameters, see `example.launch` from `hubero_bringup_gazebo_ros`.

Note that basic parameter setups for different planning algorithms were prepared (see `config/planning` directory), they just need to be selected via parameter. File suffixes can be used to assign a different parameter set to each actor.

## Usage notes

- frame names in `actor.launch` must match the frame names defined in sensors attached to the actors in Gazebo (particularly, in Gazebo world definition)

- `sensor_tf` is basically closely related to the actor sensor placement in `.world` file
  - TFs must be adjusted if actor definition in `.world` changes (there is no easy way to "read" relative poses of actor links in `ActorPlugin`)

Tested with Ubuntu 16.04 and ROS Kinetic (`rosversion: 1.12.17`).
