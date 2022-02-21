# hubero_bringup_gazebo_ros

Package contains `.launch` files, `.pgm` maps and `.world` files for evaluation of exemplary scenarios.

[Video presentation](https://vimeo.com/397552304) of the first revision of the framework.

## Run

`Parking` world, defined for Gazebo, with all necessary Actor interfaces (note that there are 4 actors in this `.world`), can be launched with:

```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=parking rviz:=true
```

If all went OK, after few seconds after start you should see `move_base` log:

```console
[ INFO] [1643660811.668211218, 8.331000000]: odom received!
```

**NOTE**: actor names defined in Gazebo `.world` must match names given to `.launch` file.

## Spawn

The crucial question is: how to spawn an `Actor` with a `ActorPlugin` controller in a custom Gazebo world?

Since Gazebo entities are defined in `.world` file, one must add Actor definition to the Gazebo world. Actor definition consists of:

- starting pose definition,
- animation definitions,
- attached sensor relative pose and definition,
- plugin used to control Actor.

Generally, one should use the exemplary worlds definitions as a reference to use actors in their custom worlds. Simply open `living_room.world` or `parking.world` and copy section that is related to a given actor:

```xml
<actor name="actor1">
  <!-- many lines of actor1 definition:
    - initial pose,
    - sensor
    - animations
    - control plugin
  -->
</actor>
```

## ROS interface

Actors will not have mobility skills without connection with ROS Navigation stack. Also, Actors will not be able to receive task requests without connection to ROS topics. Use `example.launch` parameter definitions to properly define new Actor interfaces.

## Gazebo world map

To create an ideal map of the Gazebo world one may use [`pgm_map_creator`](https://github.com/hyfan1116/pgm_map_creator) package. Examples of use are provided in `maps/*.yaml` files.

One must remember to place/uncomment `collision_map_creator` plugin definition in the `.world` file:

```xml
<plugin name='collision_map_creator' filename='libcollision_map_creator.so'/>
```

## Debugging

`example.launch` provides a possibility to debug Actor controller plugin. First, user should extend `GAZEBO_PLUGIN_PATH`:

```bash
export GAZEBO_PLUGIN_PATH=<PATH_TO_hubero_gazebo_PACKAGE>/lib:$GAZEBO_PLUGIN_PATH
```

Then, run Gazebo world with the required plugins. ROS1 distro should be adjusted by user:

```bash
gazebo -s /opt/ros/kinetic/lib/libgazebo_ros_paths_plugin.so -s /opt/ros/kinetic/lib/libgazebo_ros_api_plugin.so <PATH_TO_WORLD_FILE_DIR>/parking.world
```

ROS-part of the launch:

```bash
roslaunch hubero_bringup_gazebo_ros example.launch world:=parking rviz:=false gdb:=True
```
