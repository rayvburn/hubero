/*
 * living_room.cpp
 *
 *  Created on: Mar 9, 2020
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
	ros::init(argc, argv, "parking_scenario_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = std::stoi(argv[1]);
	}

	// create objects
	ActorAction actor1("/gazebo/actor_plugin_ros_interface/actor1", &nh);
	ActorAction actor2("/gazebo/actor_plugin_ros_interface/actor2", &nh);
	ActorAction actor3("/gazebo/actor_plugin_ros_interface/actor3", &nh);
	ActorAction actor4("/gazebo/actor_plugin_ros_interface/actor4", &nh);

	// wait
	std::this_thread::sleep_for(std::chrono::milliseconds(launch_delay));

	// =================== 1st stage ========================================
	/* Actor4 and Actor3 go straight to their cars;
	 * Actor2 goes to his friend, Actor1 goes to the dustbin */
	ROS_INFO("[SCENARIO] Firing up the 1st stage!");
	actor4.setGoal(-0.5, +7.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	actor3.setGoal(-0.5, +18.8);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	actor2.setGoal(+7.0, +2.9);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	actor1.setGoal(+0.6, -5.8);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================

	// wait for finish
	while ( !(actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) ) {
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("[SCENARIO] Firing up the 2nd stage!");

	/* Actor2 talks for few seconds */
	actor2.setStance(7); // ACTOR_STANCE_TALK_A
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	actor2.setStance(8); // ACTOR_STANCE_TALK_B
	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================

	// actor1 will surely go for a longer time than actor2
	while ( !(actor1.getClientPtrSetGoal()->getState() == State::SUCCEEDED) ) {
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	ROS_INFO("[SCENARIO] Firing up the 3rd stage!");
	/* Actor1 and Actor2 go to their cars */
	actor2.setGoal(-3.5, +10.5);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	actor1.setGoal(-0.5, +14.8);

	while ( !(actor1.getClientPtrSetGoal()->getState() == State::SUCCEEDED &&
			  actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) )
	{
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("[SCENARIO] 3rd stage completed!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	return (0);

}
