#include <ros/ros.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <hubero_ros/task_request_ros_api.h>

int main(int argc, char** argv) {
	// node initialization
	ros::init(argc, argv, "living_room_scenario_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument(s) provided
	long int launch_delay = 30000;//0
	if ( argc >= 2 ) {
		launch_delay = 1000 * std::stoi(argv[1]);
	}

	// create interfaces to request tasks from actors
	hubero::TaskRequestRosApi actor1("actor1");
	hubero::TaskRequestRosApi actor2("actor2");

	// wait
	std::this_thread::sleep_for(std::chrono::milliseconds(launch_delay));

	ROS_INFO("Scenario operation finished!");
	return 0;
}
