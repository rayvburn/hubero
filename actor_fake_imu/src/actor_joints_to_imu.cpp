/*
 * actor_joints_to_imu.cpp
 *
 *  Created on: Dec 11, 2018
 *      Author: rayvburn
 */

// --------------------------------------------------------------

#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <cmath>

#include <gazebo_msgs/LinkStates.h>

#include "ActorJointsConstants.h"

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

// --------------------------------------------------------------

/*
 * 2 IMU sensors are located on the actor's body:
 * 		- one is attached on on the belt or confirmed to lower back
 * 		- second is attached to the upper-leg on confirmed in the pocket
 * 		  (it must map the current femur orientation)
 */

// --------------------------------------------------------------

#define DEBUG_LINK_NOT_FOUND_FLAG 	999u
#define GRAVITY_STANDARD			9.80665f

// --------------------------------------------------------------

static std::vector<std::string> links_to_debug_names;
static std::vector<ActorStance> stances;

struct RPYAngles {
	tfScalar roll;
	tfScalar pitch;
	tfScalar yaw;
	RPYAngles(): roll(0.0), pitch(0.0), yaw(0.0) {}
};

static std::vector<ActorPosition> last_actor_position; 	// to measure acceleration
static tf::Matrix3x3 tf_matrix;							// to avoid allocating in callback
static tf::Quaternion quaternion;						// to avoid allocating in callback

//static RPYAngles rpy_joint;
//static RPYAngles rpy_gravity;

static tfScalar rpy_joint_angles[3] = {0, 0, 0};
static tfScalar rpy_gravity_angles[3] = {0.0, 0.0, 0.0};
static float acc_raw[3] = {0, 0, 0};

// static std::vector<std::array<tfScalar, 3> > links_rpy_joint_angles;
// static std::vector<std::array<tfScalar, 3> > links_rpy_gravity_angles;

static uint16_t cb_counter = 0;

// --------------------------------------------------------------

static uint  GetVectorIdxWithLinkName(const std::string &link_name);
void 		 LinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg);
static void  ConvertQuaternionToRPY(const tfScalar &_quat_x, const tfScalar &_quat_y, const tfScalar &_quat_z, const tfScalar &_quat_w);
static bool  TransformRPYToGravityAngles(const uint &link_id);
static void  CalculateRawAccelerations(void);
static double ChangeNumberToNegative(const double number);

// static float GetActorVelocity(const ActorPosition &last_pos);
// static float GetActorAcceleration(ActorPosition &last_pos);	// will modify last_pos

// --------------------------------------------------------------

int main(int argc, char** argv)
{

  ros::init(argc, argv, "actor_joints_to_imu");
  ros::NodeHandle nh;

  stances.push_back(WALKING_CONFIG);
  stances.push_back(STANDING_CONFIG);
  stances.push_back(SITTING_CONFIG);

  links_to_debug_names.push_back("actor1::LowerBack");
  links_to_debug_names.push_back("actor1::LeftUpLeg");

  /*
  for ( uint i = 0; i < links_to_debug_names.size(); i++ ) {

	  std::array<tfScalar, 3> arr_temp = {0.0, 0.0, 0.0};
	  links_rpy_joint_angles.push_back(arr_temp);
	  links_rpy_gravity_angles.push_back(arr_temp);

  }
	*/

  ros::Subscriber link_states_sub =  nh.subscribe("/gazebo/link_states",  3, LinkStatesCallback);

  ros::spin();
  return 0;

}

// --------------------------------------------------------------

void LinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg) {

	if ( ++cb_counter == 100 ) {
		cb_counter = 0;
	} else {
		return;
	}

	for ( uint i = 0; i < msg->name.size(); i++ ) {

		uint idx = GetVectorIdxWithLinkName(msg->name[i]);
		if ( idx == DEBUG_LINK_NOT_FOUND_FLAG ) continue;

		std::cout << "===========" << msg->name[i] << "===========" << std::endl;

		ConvertQuaternionToRPY(msg->pose[i].orientation.x, msg->pose[i].orientation.y,
							   msg->pose[i].orientation.z, msg->pose[i].orientation.w);

		std::cout << "RPY === R: " << rpy_joint_angles[0] << " P: " << rpy_joint_angles[1] << " Y: " << rpy_joint_angles[2] << std::endl;

		/* */
		if ( TransformRPYToGravityAngles(idx) == false ) {
			// sth went wrong
			std::cout << "=========== WRONG ===========" << std::endl;
			continue;
		}

		std::cout << "GRAV == R: " << rpy_gravity_angles[0] << " P: " << rpy_gravity_angles[1] << " Y: " << rpy_gravity_angles[2] << std::endl;
		CalculateRawAccelerations();
		std::cout << "ACC === x: " << acc_raw[0] << " y: " << acc_raw[1] << " z: " << acc_raw[2] << std::endl;
		std::cout << "ACC_EUCLID = " << sqrt( (acc_raw[0]*acc_raw[0]) +
											  (acc_raw[1]*acc_raw[1]) +
											  (acc_raw[2]*acc_raw[2]) )
									 << std::endl;

		/*
	 	 */

	}
}

// --------------------------------------------------------------

static uint GetVectorIdxWithLinkName(const std::string &link_name) {

	for ( size_t i = 0; i < links_to_debug_names.size(); i++ ) {
		if ( link_name == links_to_debug_names[i] ) {
			return static_cast<uint>(i);
		}
	}
	return DEBUG_LINK_NOT_FOUND_FLAG;

}

// --------------------------------------------------------------

void ConvertQuaternionToRPY(const tfScalar &_quat_x, const tfScalar &_quat_y,
							const tfScalar &_quat_z, const tfScalar &_quat_w) {

	quaternion.setValue(_quat_x, _quat_y, _quat_z, _quat_w);
	tf_matrix.setRotation(quaternion);
	tf_matrix.getRPY(rpy_joint_angles[0], rpy_joint_angles[1], rpy_joint_angles[2]);

}

bool TransformRPYToGravityAngles(const uint &link_id) {

	// rotation of coordinate systems must be taken into consideration
	// create rotation matrix

	//rpy_gravity_angles[0] = +rpy_joint_angles[0] - (M_PI / 2.00);
	//rpy_gravity_angles[1] = -rpy_joint_angles[1];
	//rpy_gravity_angles[2] = +rpy_joint_angles[2];

	if ( links_to_debug_names[link_id] == "actor1::LowerBack" ) {
		// V4
		// lower back
		rpy_gravity_angles[0] = -rpy_joint_angles[1];
		rpy_gravity_angles[1] = +rpy_joint_angles[0] - (M_PI / 2.000000);
		rpy_gravity_angles[2] = +rpy_joint_angles[2] - (M_PI / 2.000000);
		return true;

	} else if ( links_to_debug_names[link_id] == "actor1::LeftUpLeg" ) {

		rpy_gravity_angles[0] = +rpy_joint_angles[1];
		rpy_gravity_angles[1] = -rpy_joint_angles[0] - (M_PI / 2.000000);
		rpy_gravity_angles[2] = +rpy_joint_angles[2] - (M_PI / 2.000000);
		return true;

	} else {

		return false;

	}

}

void CalculateRawAccelerations() {

	// only orientation taken into consideration, not the actor's movement (his speed change)
	// V1
	//acc_raw[0] = static_cast<float>(sin(gravity_pitch) * GRAVITY_STANDARD);
	//acc_raw[1] = static_cast<float>(sin(gravity_roll)  * GRAVITY_STANDARD);
	//acc_raw[2] = static_cast<float>( (cos(gravity_roll) + sin(gravity_pitch) ) * GRAVITY_STANDARD );

	// V1 still
	//acc_raw[0] = static_cast<float>(sin(rpy_gravity_angles[1]) * GRAVITY_STANDARD);
	//acc_raw[1] = static_cast<float>(sin(rpy_gravity_angles[0])  * GRAVITY_STANDARD);
	//acc_raw[2] = static_cast<float>( (cos(rpy_gravity_angles[0]) + sin(rpy_gravity_angles[1]) ) * GRAVITY_STANDARD );
	// ^ not that bad measurements for just a standing position

	// V2 (9.8 - 11.0)
	// acc_raw[0] = static_cast<float>((-sin(rpy_gravity_angles[2]) - sin(rpy_gravity_angles[1])) * GRAVITY_STANDARD);
	// acc_raw[1] = static_cast<float>((+sin(rpy_gravity_angles[0]) + sin(rpy_gravity_angles[2])) * GRAVITY_STANDARD);
	// acc_raw[2] = static_cast<float>((+cos(rpy_gravity_angles[0]) + sin(rpy_gravity_angles[1])) * GRAVITY_STANDARD );

	// V3 (15.0 - 30.0)
	// acc_raw[0] = static_cast<float>((+cos(rpy_gravity_angles[2]) + sin(rpy_gravity_angles[1])) * GRAVITY_STANDARD);
	// acc_raw[1] = static_cast<float>((+sin(rpy_gravity_angles[2]) + sin(rpy_gravity_angles[0])) * GRAVITY_STANDARD);
	// acc_raw[2] = static_cast<float>((+cos(rpy_gravity_angles[0]) + cos(rpy_gravity_angles[1])) * GRAVITY_STANDARD );

	// V4 ( 9.60 - 9.90)
	/*
	acc_raw[0] = static_cast<float>(( +sin(rpy_gravity_angles[1])*cos(rpy_gravity_angles[2]) +
									   sin(rpy_gravity_angles[0])*sin(rpy_gravity_angles[2]) ) * -GRAVITY_STANDARD);
	acc_raw[1] = static_cast<float>(( +sin(rpy_gravity_angles[0])*cos(rpy_gravity_angles[2]) +
									   sin(rpy_gravity_angles[1])*sin(rpy_gravity_angles[2]) ) * -GRAVITY_STANDARD);
	acc_raw[2] = static_cast<float>(( +cos(rpy_gravity_angles[0])*cos(rpy_gravity_angles[1]) ) * -GRAVITY_STANDARD);
	*/

	// V5
	acc_raw[0] = static_cast<double>(( +sin(rpy_gravity_angles[1])*cos(rpy_gravity_angles[2]) +
									    sin(rpy_gravity_angles[0])*sin(rpy_gravity_angles[2]) ) * GRAVITY_STANDARD);
	acc_raw[1] = static_cast<double>(( +sin(rpy_gravity_angles[0])*cos(rpy_gravity_angles[2]) +
									    sin(rpy_gravity_angles[1])*sin(rpy_gravity_angles[2]) ) * GRAVITY_STANDARD);
	acc_raw[2] = static_cast<double>(( +cos(rpy_gravity_angles[0])*cos(rpy_gravity_angles[1]) ) * GRAVITY_STANDARD);

	/* TODO: if possible - avoid below extra operations by refining above equations
	 * raw accelerations (gravity effect only) will always be negative */
	acc_raw[0] = ChangeNumberToNegative(acc_raw[0]);
	acc_raw[1] = ChangeNumberToNegative(acc_raw[1]);
	acc_raw[2] = ChangeNumberToNegative(acc_raw[2]);



}

double ChangeNumberToNegative(const double number) {

	if ( number >= 0.00000 ) {
		return -number;
	} else {
		return number;
	}

}




