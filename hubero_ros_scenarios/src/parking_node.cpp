#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_common/defines.h>
#include <hubero_ros/task_request_ros_api.h>

using namespace hubero;

const std::string TF_FRAME_REF = "world";

/// Allows to process any ROS callbacks
void waitRefreshingRos() {
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	ros::spinOnce();
}

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "parking_scenario_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 0;
	if ( argc >= 2 ) {
		launch_delay = 1000 * std::stoi(argv[1]);
	}

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");
	hubero::TaskRequestRosApi actor3("actor3");
	hubero::TaskRequestRosApi actor4("actor4");

	// wait
	ROS_INFO("TaskRequestRos APIs fired up, scenario execution will start in %lu seconds", launch_delay / 1000);
	std::this_thread::sleep_for(std::chrono::milliseconds(launch_delay));

	// =================== 1st stage ========================================
	/* Actor4 and Actor3 go straight to their cars;
	 * Actor2 goes to his friend, Actor1 goes to the dustbin */
	ROS_INFO("[SCENARIO] Firing up the 1st stage!");

	actor4.moveToGoal(Vector3(+0.5, +6.6, 0.0), IGN_PI, TF_FRAME_REF);
	actor3.moveToGoal(Vector3(+0.3, +18.8, 0.0), IGN_PI, TF_FRAME_REF);
	actor2.moveToGoal(Vector3(+7.0, +2.9, 0.0), 0.0, TF_FRAME_REF);
	actor1.moveToGoal(Vector3(+1.2, -5.7, 0.0), IGN_PI, TF_FRAME_REF);

	ROS_INFO("[SCENARIO] 1st stage completed!");

	// =================== 2nd stage ========================================
	// we will be waiting for actor2's task finish, so we must also know if the task request was processed
	if (actor2.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
		while (actor2.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
			if (!ros::ok()) {
				ROS_INFO("Node stopped!");
				return (0);
			}
			ROS_INFO("Waiting for Actor2 task to become active. Current state %d...", actor2.getMoveToGoalState());
			waitRefreshingRos();
		}
	}

	ROS_INFO("Task of Actor2 is active! Waiting until he finishes 'moveToGoal'");
	while (actor2.getMoveToGoalState() == TASK_FEEDBACK_ACTIVE) {
		if (!ros::ok()) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		waitRefreshingRos();
	}

	ROS_INFO("[SCENARIO] Firing up the 2nd stage!");

	/* Actor2 talks for few seconds */
	ROS_INFO("Actor2 got to the place! He will start talking to the person nearby!");
	actor2.talk(Vector3(+7.0, +2.9, 0.0), IGN_DTOR(90), TF_FRAME_REF);

	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	ros::spinOnce();

	ROS_INFO("Actor2 finishes talking to the person nearby!");
	actor2.stopTalking();

	ROS_INFO("[SCENARIO] 2nd stage completed!");

	// =================== 3rd stage ========================================

	// actor1 will surely go for a longer time than actor2
	if (actor1.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
		while (actor1.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
			if (!ros::ok()) {
				ROS_INFO("Node stopped!");
				return (0);
			}
			ROS_INFO("Waiting for Actor1 task to become active. Current state %d...", actor2.getMoveToGoalState());
			waitRefreshingRos();
		}
	}

	ROS_INFO("Task of Actor1 is active! Waiting until he finishes 'moveToGoal'");
	while (actor1.getMoveToGoalState() == TASK_FEEDBACK_ACTIVE) {
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		waitRefreshingRos();
	}
	ROS_INFO("Actor1 threw the rubbish away to the dumpster!");

	ROS_INFO("[SCENARIO] Firing up the 3rd stage!");

	/* Actor1 and Actor2 go to their cars */
	ROS_INFO("Actor1 and Actor2 start moving towards their cars!");
	actor2.moveToGoal(Vector3(-3.5, +10.5, 0.0), IGN_PI, TF_FRAME_REF);
	actor1.moveToGoal(Vector3(-0.5, +14.8, 0.0), IGN_PI, TF_FRAME_REF);

	// wait until request processed
	if (actor1.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE	|| actor2.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
		while (actor1.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE || actor2.getMoveToGoalState() != TASK_FEEDBACK_ACTIVE) {
			if (!ros::ok()) {
				ROS_INFO("Node stopped!");
				return (0);
			}
			ROS_INFO(
				"Waiting for Actor1 or Actor2 task to become active. Current state: A1 %d, A2 %d...",
				actor1.getMoveToGoalState(),
				actor2.getMoveToGoalState()
			);
			waitRefreshingRos();
		}
	}

	ROS_INFO("Actor1 and Actor2 are moving towards cars! Waiting until they reach their goal");
	while (
		actor1.getMoveToGoalState() == TASK_FEEDBACK_ACTIVE
		|| actor2.getMoveToGoalState() == TASK_FEEDBACK_ACTIVE
	) {
		if ( !ros::ok() ) {
			ROS_INFO("Node stopped!");
			return (0);
		}
		waitRefreshingRos();
	}

	ROS_INFO("[SCENARIO] 3rd stage completed!");

	// ==================== finish ==========================================

	ROS_INFO("Scenario operation finished!");
	return 0;
}
