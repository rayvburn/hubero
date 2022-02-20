# hubero_ros_msgs

ROS messages for `hubero_ros` package.

This package contains action definitions for HuBeRo actor tasks. Actors report feedback executing task so it's a natural way to use `Action Server` and `Action Client` to report task execution progress to ROS.

Additionally, a `Person` message was defined. It aim to be broadcasted as Actor status. The `Person` message composes multiple approaches to human-related message structures so it can be easily adaptable to existing systems.

To view Person message details, run (with ROS source'd):

```bash
rosmsg show hubero_ros_msgs/Person
```
