/*
 * living_room.cpp
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <actor_sim_api/ActorAction.h>
#include <actionlib/client/simple_client_goal_state.h>

typedef actionlib::SimpleClientGoalState State;

// -------------------------------------------------------------------------------------------
// main --------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "living_room_scenario_node");
	ros::NodeHandle nh;

	ActorAction actor1("/gazebo/actor_plugin_ros_interface/actor1", &nh);
	ActorAction actor2("/gazebo/actor_plugin_ros_interface/actor2", &nh);

	// =================== 1st stage ========================================
	actor1.setGoal(+1.0, +1.0);
	actor2.setGoal(+4.0, +3.0);

	ROS_INFO("Waiting for the 1st synchronization!");
	// wait for finish of both actions - synchronization point
	while ( !(actor1.getClientPtrSetGoal()->getState() == State::SUCCEEDED &&
			  actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) )
	{
		if ( ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("1st synchronization!");

	// =================== 2nd stage ========================================
	actor1.setGoal(-1.0, -1.0);
	actor2.setGoal(+4.0, -1.0);

	ROS_INFO("Waiting for the 2nd synchronization!");
	// wait for finish of both actions - synchronization point
	while ( !(actor1.getClientPtrSetGoal()->getState() == State::SUCCEEDED &&
			  actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) )
	{
		if ( ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("2nd synchronization!");

	// =================== 3rd stage ========================================
	actor1.setGoal(+3.0, -2.0);
	actor2.setGoal(-3.0, -2.0);

	ROS_INFO("Waiting for the 3rd synchronization!");
	// wait for finish of both actions - synchronization point
	while ( !(actor1.getClientPtrSetGoal()->getState() == State::SUCCEEDED &&
			  actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) )
	{
		if ( ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("3rd synchronization!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	ros::spin(); // optional, can safely finish
	return (0);

}
