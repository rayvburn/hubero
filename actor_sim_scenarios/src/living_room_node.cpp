/*
 * living_room.cpp
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

#include <ros/ros.h>
#include <string>
#include <iostream>

#include <actor_sim_api/ActorAction.h>

// -------------------------------------------------------------------------------------------
// main --------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "living_room_scenario_node");
	ros::NodeHandle nh;

	ActorAction actor1("/gazebo/actor_plugin_ros_interface/actor1", &nh);
	ActorAction actor2("/gazebo/actor_plugin_ros_interface/actor2", &nh);

	actor1.setGoal(1.0, 1.0);

	ros::spin();
	return (0);

}
