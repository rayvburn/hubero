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
//	ros::NodeHandle nh2;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = std::stoi(argv[1]);
	}

	// create objects
	ActorAction actor1("/gazebo/actor_plugin_ros_interface/actor1", &nh);
	ActorAction actor2("/gazebo/actor_plugin_ros_interface/actor2", &nh);

	// wait
	std::this_thread::sleep_for(std::chrono::milliseconds(launch_delay));

	// =================== init stage ========================================
//	actor1.setStance(3); // ACTOR_STANCE_SIT_DOWN
//	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//	actor1.setStance(4); // ACTOR_STANCE_SITTING
//	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//	actor1.setStance(5); // ACTOR_STANCE_STAND_UP
//	std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	// =================== 1st stage ========================================
	/* Actor2 goes to the sofa neighbourhood (Actor1 pose) */
	actor2.setGoal(+5.0, -1.0);

	ROS_INFO("[SCENARIO] Firing up the 1st stage!");
	// wait for finish of both actions - synchronization point
	while ( !(actor2.getClientPtrSetGoal()->getState() == State::SUCCEEDED) ) {
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================
	/* Actor2 talks for few seconds */
	ROS_INFO("[SCENARIO] Firing up the 2nd stage!");
	actor2.setStance(7); // ACTOR_STANCE_TALK_A
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	actor2.setStance(8); // ACTOR_STANCE_TALK_B
	std::this_thread::sleep_for(std::chrono::milliseconds(4000));
	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================
	/* Both Actor1 and Actor2 go to the big table */
	ROS_INFO("[SCENARIO] Firing up the 3rd stage!");
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

//	actor1.setGoalName("table_conference_2");
	actor2.setGoal(+2.0, -2.7);
	// actor1.setGoal(+2.5, -4.8);
	// actor1.setGoal(+2.5, -5.3);
	// OK, far // actor1.setGoal(+2.7, -5.1);
	// NEIN // actor1.setGoal(+1.5, -4.8);

	// wait until Actor1 finish rotation (roughly estimated)
//	std::this_thread::sleep_for(std::chrono::milliseconds(1500)); // diagram does not consider that
//	actor2.setGoalName("table_conference_2"); // does not work
//	actor2.setGoal(+2.5, -4.8); // does not work
//	actor2.setGoal(+2.1, -0.1); // OK, far
//	actor2.setGoal(+0.1, -1.0); // OK
	actor1.setGoal(+0.5, -1.2); // 1.0 ok
	// NEIN // actor2.setGoal(+1.5, -3.2);

	ROS_INFO("Waiting for the synchronization!");
	// wait for finish of both actions - synchronization point
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
