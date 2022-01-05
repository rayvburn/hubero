# hubero_ros
ROS interface for A Framework to Simulate Human Behaviour in Robot Research

## Main features

How does this package cooperates with HuBeRo?
- each Actor in HuBeRo dynamically loads `NavigationROS` (which extends `NavigationBase`)

How is this package connected to ROS Navigation stack?
- `NavigationROS` connects to `move_base` topics - this allows to pass navigation commands to each actor

Actor messages are published to ROS via `NavigationROS` plugin.

## Usage notes
- frame names from `move_base.launch` must match the frame names defined in sensors attached to the actors in Gazebo (particularly, in Gazebo world definition)
- `actor_namespace` from `planning_ros.launch` must be equal to name of the actor created in `Gazebo` simulation
  - name of the actor defines namespace of navigation part of the system that runs in ROS as `move_base` node (one for each actor)
