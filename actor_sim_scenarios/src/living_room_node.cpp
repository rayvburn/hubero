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
#include "../../actor_plugin_social/include/actor/core/Enums.h" // FIXME

typedef actionlib::SimpleClientGoalState State;

// -------------------------------------------------------------------------------------------
// main --------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "living_room_scenario_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = 1000 * std::stoi(argv[1]);
	}

	// create objects
	ActorAction actor1("/gazebo/actor_plugin_ros_interface/actor1", &nh);
	ActorAction actor2("/gazebo/actor_plugin_ros_interface/actor2", &nh);

	// wait //
	std::this_thread::sleep_for(std::chrono::milliseconds(launch_delay));

	// =================== 1st stage ========================================
	/* Actor2 goes to the table neighbourhood (Actor1 pose) */
	actor2.setGoal(+5.5, +1.0);

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
	/* Actor2 talks for a few seconds */
	ROS_INFO("[SCENARIO] Firing up the 2nd stage!");
	actor2.setStance(actor::ActorStance::ACTOR_STANCE_TALK_B);
	std::this_thread::sleep_for(std::chrono::milliseconds(4000));
	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================
	/* Both Actor1 and Actor2 go to the big table */
	ROS_INFO("[SCENARIO] Firing up the 3rd stage!");

//	actor2.setGoal(+0.1, -2.1); // +2.0, -2.8 //   // +1.5, -2.2 //  // (+2.0, -4.8);
	actor2.setGoal(+2.0, -2.8);
	// wait until Actor1 finishes rotation (roughly estimated);
	// NOTE: scenario diagram does not consider that! but it is essential
	// because the `social force` starts to be immediately generated after
	// Actor2 rotation finish which drives Actor1 into opposite direction!
	// which looks suspicious
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//	actor1.setGoal(+2.1, -2.0); // +0.6, -2.1 // //actor1.setGoal(+1.5, -3.2);
	actor1.setGoal(+0.6, -2.0);

	ROS_INFO("[SCENARIO] Waiting for the synchronization!");
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
