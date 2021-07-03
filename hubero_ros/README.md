# hubero_ros
ROS interface for A Framework to Simulate Human Behaviour in Robot Research

# Usage notes
- frame names from `move_base.launch` must match to the frame names defined in sensors attached to the actors in Gazebo
- actor frames are published via `NavigationROS` plugin
- `actor_namespace` from `planning_ros.launch` must be equal to name of the actor created in `Gazebo` simulation
	- name of the actor defines namespace of navigation part of the system that runs in ROS as `move_base` node (one for each actor)