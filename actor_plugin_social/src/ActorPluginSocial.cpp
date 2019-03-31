/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>
#include <tgmath.h>		// fabs()

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorPluginSocial.h"
#include "print_info.h"

bool print_info = false;

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

// ==========================================================================================
// ========================= CRASH / BUG / FAIL (?) note ====================================
// ==========================================================================================
/* static std::vector creates some crash (couldn't find anything wrong in .log)				=
 * every operation on the vector (even try of printing the address) causes:					=
 * 		o  	actor's pose calculation is completely different compared to the one			=
 * 			presented in the plugin's code,													=
 *		o	no std::cout debug info is printed (even placed in the plugin's constructor)	=
 *		o 	pure declaration of the vector does not do any harm								=
 * As a workaround there is a global vector of the same type in class' .cpp file			=
 * Below was in the header:																	=
 * //	private: static std::vector<ignition::math::Vector3d> lin_vels_vector;				=
 *///																						=
/* static std::map - same as above															=
	 * // 	private: static std::map<std::string, unsigned int> map_of_names; *///			=
// ==========================================================================================
/*
std::vector<ignition::math::Vector3d> lin_vels_vector;
std::map<std::string, unsigned int> map_of_names;
*/

#if defined(CREATE_ROS_NODE) || defined(CREATE_ROS_INTERFACE)
#include <visualization_msgs/MarkerArray.h>
#endif

#define SILENT_

#ifdef VISUALIZE_SFM
	#ifdef VIS_SFM_POINT
	SocialForceModel::SFMVisPoint ActorPlugin::sfm_vis;
	#elif defined(VIS_SFM_GRID)
	SocialForceModel::SFMVisGrid ActorPlugin::sfm_vis;
	#endif
	SocialForceModel::SFMVisGrid ActorPlugin::grid_vis;
#endif

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
	std::cout << "CONSTRUCTOR HERE, hello" << std::endl;
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	std::cout << "ACTOR PLUGIN::LOAD!" << std::endl;
//	std::cout << "LOAD()   lin vels address" << &lin_vels_vector << std::endl;

	this->sdf = _sdf;
	this->model = _model;
	this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
	this->world = this->actor->GetWorld();
	this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
		  std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));
	this->Reset();

	if ( this->ReadSDF() ) { /* TODO: Exception Handling */ }

	// - - - - - - - - - - - - - - - - - - - - - -  - - - -- - - -- - - -- -  -- -

#ifdef CREATE_ROS_NODE

	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = nullptr;
		ros::Init(argc, argv, "gazebo_ros", ros::init_options::NoSigintHandler);
	}

	ros_nh.reset(new ros::NodeHandle());
	vis_pub = ros_nh->advertise<visualization_msgs::MarkerArray>("sfm_mrkr", 1000);

	if(!vis_pub) {
		ROS_FATAL_STREAM("Unable to create publisher for topic ``sfm_mrkr``");
	}

#elif defined(CREATE_ROS_INTERFACE)
	ros_interface.Init(this->actor->GetName());
#endif

#ifdef VISUALIZE_SFM
	sfm_vis.Init("sfm", "map");
	grid_vis.Init("sfm_grid", "map");
#endif

	// - - - - - - - - - - - - - - - - - - - - - -  - - - -- - - -- - - -- -  -- -
    std::cout << "LOADED POSE: " << this->actor->WorldPose() << std::endl;

  	ignition::math::Vector3d init_orient = this->actor->WorldPose().Rot().Euler();
  	ignition::math::Pose3d init_pose;
  	ignition::math::Vector3d init_position;
  	init_position = this->actor->WorldPose().Pos();
  	init_position.Z(1.2138);

  	init_pose.Set(init_position,
				  ignition::math::Quaterniond(init_orient.X() + IGN_PI/2,
											  init_orient.Y(),
											  // init_orient.Z() + 1.5707));
											  init_orient.Z()));

	// WARNING: initial pose changed!
  	this->actor->SetWorldPose(init_pose, false, false);
	std::cout << " -------- SET WorldPose() actor! -------- " << init_pose << std::endl;

	// conversions between Euler and Quaternion will finally produce the result that converges to 0...
	// above is deprecated, yaw will be set in each OnUpdate()

	// Set last_pos_actor to prevent velocity shootout
	last_pose_actor.Pos() = this->actor->WorldPose().Pos();
	std::cout << " -------- SET last_pos_actor! -------- " << last_pose_actor.Pos() << std::endl;

	actor_common_info.AddActor(this->actor->GetName());

#if	defined(INFLATE_BOUNDING_BOX)
	actor_common_info.SetBoundingBox( this->GenerateBoundingBox(this->pose_actor) );
#elif defined(INFLATE_BOUNDING_CIRCLE)
	bounding_circle.SetCenter(this->pose_actor.Pos());
	bounding_circle.SetRadius(0.75f);
	actor_common_info.SetBoundingCircle(bounding_circle);
#elif defined(INFLATE_BOUNDING_ELLIPSE)

#endif


	std::cout << " -------- ACTOR ID -------- " << actor_common_info.GetActorID() << std::endl;
	std::cout << " -------- MODEL TYPE -------- " << this->model->GetType() << std::endl;

	// WARNING: HARD-CODED target coord
	if ( this->actor->GetName() == "actor1" ) {
		this->target.X(+0.00);
		this->target.Y(-4.00);
	}

	prev_state_actor = ACTOR_STATE_MOVE_AROUND;
	sfm.Init(80.0, 2.0, 1.0, this->world);

}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity_desired = 0.8;
  this->last_update = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    stance_actor = ACTOR_STANCE_WALK;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////

bool ActorPlugin::isTargetStillReachable(const common::UpdateInfo &_info) {

	// check periodically, no need to do this in each iteration
	if ( (_info.simTime - this->last_reachability_time).Double() > 2.0 ) {

		// save event time
		this->last_reachability_time = _info.simTime;

		// iterate over all models
		for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {

			// FIXME: cafe is a specific model that represents whole world
			if ( this->world->ModelByIndex(i)->GetName() == "cafe" ) {
				continue;
			}

			// check if model's bounding box contains target point
			if ( this->doesBoundingBoxContainPoint(this->world->ModelByIndex(i)->BoundingBox(), this->target) ) {

				std::cout << "isTargetStillReachable()" << std::endl;
				std::cout << "\t" << this->actor->GetName() << "\tDETECTED TARGET UNREACHABLE!" << std::endl;
				std::cout << "\ttarget: " << this->target << "\tmodel containing: " << this->world->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				std::cout << std::endl;
				std::cout << std::endl;
				return (false);

			}

		}

	}

	return (true);
}

/////////////////////////////////////////////////

bool ActorPlugin::isTargetNotReachedForTooLong(const common::UpdateInfo &_info) const {

	// TODO: make the time a parameter
	if ( (_info.simTime - this->last_target_selection_time).Double() > 60.0 ) {

		std::cout << "isTargetNotReachedForTooLong()" << std::endl;
		std::cout << "\t" << this->actor->GetName() << "\tDETECTED TARGET UNREACHABLE IN FINITE TIME!" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		return (true);

	}
	return (false);

}

/////////////////////////////////////////////////

bool ActorPlugin::doesBoundingBoxContainPoint(const ignition::math::Box &_bb, const ignition::math::Vector3d &_pt) const {

	// check if model's bounding box is valid (not 0-length - for actors it is - || NaN || inf)
	if ( !std::isnan(_bb.Max().Length()) && !std::isinf(_bb.Max().Length()) && (_bb.Max().Length() > 1e-06) ) {

		// check if model's bounding box contains target point
		if ( _bb.Contains(_pt) ) {
			return (true);
		}

	}
	return (false);

}

/////////////////////////////////////////////////

void ActorPlugin::chooseNewTarget(const common::UpdateInfo &_info) {

	/* TODO: periodically check if current target is still available
	 * for example: new model added in Gazebo */
	ignition::math::Vector3d new_target(this->target);

	// look for target that is located at least 2 meters from current one
	while ((new_target - this->target).Length() < 2.0) {

		// get random coordinates based on world limits
		// TODO: world limits loaded as a parameter
		new_target.X(ignition::math::Rand::DblUniform(-3, 3.5));
		new_target.Y(ignition::math::Rand::DblUniform(-10, 2));

		// check distance to all world's objects
		for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {

			/* distance-based target selection - could fail for very big objects
			 *
			//double dist = (this->world->ModelByIndex(i)->WorldPose().Pos() - newTarget).Length();
			if (dist < 2.0) {
				// if distance to some object is less than 2 meters
				// discard - discard current target and look
				// for another one
				newTarget = this->target;
				break;
			}
			*
			*/

			/* bounding-box-based target selection - more safe for big obstacles,
			 * accounting some tolerance for a target accomplishment - an actor should
			 * not step into an object */

			// FIXME: cafe is a specific model that represents whole world
			if ( this->world->ModelByIndex(i)->GetName() == "cafe" ) {
				continue;
			}

			// check if model's bounding box contains target point
			if ( this->doesBoundingBoxContainPoint(this->world->ModelByIndex(i)->BoundingBox(), this->target) ) {
				// TODO: make this an error log message
				std::cout << "chooseNewTarget() - selection failed -> model containing target's pos: " << this->world->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				new_target = this->target;
				break;
			}

		} // for

	} // while

	// finally found a new target
	this->target = new_target;

	// save event time
	last_target_selection_time = _info.simTime;

}


/////////////////////////////////////////////////


#if	defined(INFLATE_BOUNDING_BOX)

ignition::math::Box ActorPlugin::GenerateBoundingBox(const ignition::math::Pose3d &_actor_pose) {

	// lengths expressed in actor's coordinate system - x-axis is pointing forward (from face)
	static const double ACTOR_X_BB_HALF_LENGTH = 0.45;
	static const double ACTOR_Y_BB_HALF_LENGTH = 0.45;
	static const double ACTOR_Z_BB_HALF_LENGTH = 1.0;

	// forced calculations in 0-90 deg range
	double yaw_truncated = std::fabs(_actor_pose.Rot().Yaw());
	yaw_truncated -= static_cast<int>(yaw_truncated / IGN_PI_2) * IGN_PI_2;

	/* TODO: Add roll and pitch rotation handling (as actor lies down X and Z changes);
	 * 		 it won't be used in move_around state but at last will be nice
	 * 		 to have such a feature */
//	double roll_truncated = std::fabs(_actor_pose.Rot().Roll());
//	roll_truncated -= static_cast<int>(roll_truncated / IGN_PI_2) * IGN_PI_2;
//
//	double pitch_truncated = std::fabs(_actor_pose.Rot().Pitch());
//	pitch_truncated -= static_cast<int>(pitch_truncated / IGN_PI_2) * IGN_PI_2;

	// x-projection on x-axis
	double xp = ACTOR_X_BB_HALF_LENGTH * cos(yaw_truncated);
	// line projected on x-axis that extends basic x-projection
	double xp_ext = ACTOR_Y_BB_HALF_LENGTH * sin(yaw_truncated);
	// sum of xp's gives total length of x-component of BB
	double x_total = xp + xp_ext;

	// the same for y-axis
	double yp = ACTOR_Y_BB_HALF_LENGTH * cos(yaw_truncated);
	double yp_ext = ACTOR_X_BB_HALF_LENGTH * sin(yaw_truncated);
	double y_total = yp + yp_ext;

	ignition::math::Box bb(	_actor_pose.Pos().X() - x_total,
							_actor_pose.Pos().Y() - y_total,
							_actor_pose.Pos().Z() - ACTOR_Z_BB_HALF_LENGTH,
							_actor_pose.Pos().X() + x_total,
							_actor_pose.Pos().Y() + y_total,
							_actor_pose.Pos().Z() + ACTOR_Z_BB_HALF_LENGTH );

	if ( (bb.Max().X() - bb.Min().X()) < 0.90 || (bb.Max().X() - bb.Min().X()) > 1.272793 ) {
		std::cout << "\tBB min: " << bb.Min() << "\tmax: " << bb.Max() << std::endl;
		std::cout << "\t\tcenter: " << bb.Center() << "\tRAW x: " << _actor_pose.Pos().X() - ACTOR_X_BB_HALF_LENGTH << "\tRAW y: " << _actor_pose.Pos().Y() + ACTOR_Y_BB_HALF_LENGTH << std::endl;
		std::cout << "\t\tyaw: " << _actor_pose.Rot().Yaw() << std::endl;
		std::cout << "\t\txp: " << xp << "\txp_ext: " << xp_ext << "\tx_total: " << x_total << std::endl;
		std::cout << "\t\typ: " << yp << "\typ_ext: " << yp_ext << "\ty_total: " << y_total << std::endl;
		std::cout << "\t\tlen: " << bb.Max().X() - bb.Min().X() << std::endl;
		std::cout << "\t\tWRONG LENGTH\n\n" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
	}

	return (bb);

}
#endif


/////////////////////////////////////////////////

void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{

	bool to_start = true;
	if (_info.simTime.Double() < 8.0f ) {
		static int last_sec = 0;
		int curr_sec = static_cast<int>(_info.simTime.Double());
		if ( curr_sec != last_sec ) {
			std::cout << curr_sec << "\tWAITING..." << std::endl;
			last_sec = curr_sec;

		}
		to_start = false;
		state_actor = ACTOR_STATE_ALIGN_TARGET;
	}

	if ( !to_start ) {
		return;
	};


	if ( !isTargetStillReachable(_info) ) {
		this->chooseNewTarget(_info);
		// after setting new target, first let's rotate to its direction
		state_actor = ACTOR_STATE_ALIGN_TARGET;
	}

	if ( isTargetNotReachedForTooLong(_info) ) {
		this->chooseNewTarget(_info);
		// after setting new target, first let's rotate to its direction
		state_actor = ACTOR_STATE_ALIGN_TARGET;
	}


#ifdef CREATE_ROS_NODE
	PublishActorTf();
#elif defined(CREATE_ROS_INTERFACE)
	ros_interface.PublishActorTf(this->pose_actor);
	#ifdef INFLATE_BOUNDING_CIRCLE
	ros_interface.PublishMarker(bounding_circle.GetMarkerConversion());
	#elif defined(INFLATE_BOUNDING_BOX)
	ros_interface.PublishMarker(sfm_vis.GetBBMarkerConversion(actor_common_info.GetBoundingBox()));
	#endif
#endif

#if defined(VISUALIZE_SFM) && defined(VIS_SFM_POINT)
	static common::Time vis_time;
	static int counter = 0;

	if ( (_info.simTime - vis_time).Double() > 0.25 ) {

		//std::cout << "ACTOR FOR VIS: " << this->actor_id << "\tname: " << this->actor->GetName() << std::endl;
		VisualizeForceField();
		counter++;
		if ( counter == 2 ) {
			vis_time = _info.simTime;
			counter = 0;
		}

	}
#elif defined(VISUALIZE_SFM) && defined(VIS_SFM_GRID)

	static common::Time vis_time;
	if ( (_info.simTime - vis_time).Double() > 0.05 ) {
		VisualizeForceField();
		vis_time = _info.simTime;
	}

#endif

#ifdef SILENT_
	print_info = false;
#endif

	switch(state_actor) {

	case(ACTOR_STATE_ALIGN_TARGET):

			if ( prev_state_actor != ACTOR_STATE_ALIGN_TARGET ) {
#ifndef SILENT_
				std::cout << "\tACTOR_STATE_ALIGN_TARGET" << std::endl;
#endif
				prev_state_actor =  ACTOR_STATE_ALIGN_TARGET;
			}

			ActorStateAlignTargetHandler(_info);
			break;

	case(ACTOR_STATE_MOVE_AROUND):

			if ( prev_state_actor != ACTOR_STATE_MOVE_AROUND ) {
#ifndef SILENT_
				std::cout << "\tACTOR_STATE_MOVE_AROUND" << std::endl;
#endif
				prev_state_actor =  ACTOR_STATE_MOVE_AROUND;
			}

			ActorStateMoveAroundHandler(_info);
			break;

	case(ACTOR_STATE_STOP_AND_STARE):
#ifndef SILENT_
			std::cout << "\tACTOR_STATE_STOP_AND_STARE" << std::endl;
#endif
			break;
	case(ACTOR_STATE_FOLLOW_OBJECT):
#ifndef SILENT_
			std::cout << "\tACTOR_STATE_FOLLOW_OBJECT" << std::endl;
#endif
			ActorStateFollowObjectHandler(_info);
			break;
	case(ACTOR_STATE_TELEOPERATION):
#ifndef SILENT_
			std::cout << "\tACTOR_STATE_TELEOPERATION" << std::endl;
#endif
			ActorStateTeleoperationHandler(_info);
			break;
	}


	// debugging purposes

  	static common::Time print_time;

  	if ( (_info.simTime - print_time).Double() > 0.2 ) {

#ifndef SILENT_
		if ( this->actor->GetName() == "actor1" ) {

			print_info = true;
			Print_Set(true);
			print_time = _info.simTime;
			std::cout << "\n\n**************************************************** ACTOR1 ***********************************************" << std::endl;
			std::cout << "**** INITIAL pose: " << this->actor->WorldPose() << "\t\t actor1 velocity: \t" << this->velocity_actual << "\t target: " << this->target << std::endl;
			ignition::math::Angle yaw_from_vel_world( std::atan2(this->velocity_actual.Y(), this->velocity_actual.X()) + (IGN_PI/2) );
			yaw_from_vel_world.Normalize();
			std::cout << "**** YAW vs VEL comparison\t\tyaw_from_vel_act: " << std::atan2(this->velocity_actual.Y(), this->velocity_actual.X()) << "\tyaw_from_vel_vec: " << std::atan2( lin_vels_vector[this->actor_id].Y(), lin_vels_vector[this->actor_id].X() ) << "\tyaw_from_vel_WORLD: " << yaw_from_vel_world.Radian() << "\tyaw_from_pose: " << this->actor->WorldPose().Rot().Euler().Z() << std::endl;
		}
#endif

	} else {
		print_info = false;
	}

  	return;





#ifndef REFACTOR_COMMON

  	// OnUpdate algorithm
  	double dt = (_info.simTime - this->last_update).Double();
  	// ignition::math::Pose3d pose = this->actor->WorldPose();

  	// TODO: avoid the copy?
  	this->SetActorPose(this->actor->WorldPose());

  	ignition::math::Vector3d to_target_vector = this->target - this->pose_actor.Pos();

  	// ignition::math::Vector3d rpy = pose.Rot().Euler();
  	ignition::math::Vector3d rpy = this->UpdateActorOrientation();

  	CalculateVelocity(this->pose_actor.Pos(), dt);

  	/* Setting the linear velocity for actor or actor's model HAS NO EFFECT, don't know why,
  	 * couldn't find source files on disk and based on bitbucket's Gazebo sources all seems
  	 * to be fine (32 joints detected so link pointer is not NULL)
  	 * Thus a workaround with static std::vector that stores all actor's velocities */

  	// below causes no movement for both actors, only rotation
  	SetActorsLinearVel(this->actor_id, this->velocity_actual);

//  	if ( this->actor->GetName() == "actor1" ) {
//  		SetActorsLinearVel(this->actor_id, ignition::math::Vector3d(1.0,1.0,0.0));
//  	} else if ( this->actor->GetName() == "actor2" ) {
//  		SetActorsLinearVel(this->actor_id, ignition::math::Vector3d(2.0,2.0,0.0));
//  	}

  	/*
  	this->actor->SetEnabled(true);
	this->actor->SetLinearVel(this->velocity_actual);

//	std::cout << "\nACTOR GetLinks().size: " << this->model->GetLinks().size() << std::endl;
//	std::cout << "BEFORE SetLinearVel()  10.0, 20.0, 30.0" << std::endl; //  << this->velocity_actual << std::endl;

	this->model->SetLinearAccel(ignition::math::Vector3d(10.0, 20.0, 30.0));
	this->model->SetLinearVel(this->velocity_actual);
	this->model->SetWorldTwist(this->velocity_actual, ignition::math::Vector3d(0.0, 0.0, 0.0), true);
	this->model->Update();
//	std::cout << "AFTER  ACTOR SetLinearVel()  " << this->actor->WorldLinearVel() << std::endl;
//	std::cout << "AFTER  ACTOR SetLinearVel()  " << this->model->WorldLinearVel() << std::endl;

	//this->actor->WorldLinearVel();
	//this->WorldLinearVel();

//	this->world->ModelByName(this->actor->GetName())->SetLinearVel(ignition::math::Vector3d(10.0, 20.0, 30.0));
//	this->world->ModelByName(this->actor->GetName())->Update();
//	std::cout << "AFTER  SetLinearVel()  " << this->world->ModelByName(this->actor->GetName())->WorldLinearVel() << std::endl;
	*/

  	this->actor->SetLinearVel(this->velocity_actual);


	ignition::math::Vector3d sf = sfm.GetSocialForce(this->world,
													 this->actor->GetName(),
													 this->pose_actor,
													 this->velocity_actual,
													 this->target, // );
													 map_of_names,
													 lin_vels_vector,
													 bounding_boxes_vector);

	if ( print_info ) {
		std::cout << "\t TOTAL force: " << sf << std::endl;
//		std::cout << "\t\t\t Vels vector: ";
//		for ( int i = 0; i < lin_vels_vector.size(); i++ ) {
//			std::cout << "\t" << lin_vels_vector[i];
//		}
//		std::cout << std::endl;
		std::cout << "***********************  NEW_POSE_CALC  **************************" << std::endl;
	}

	ignition::math::Pose3d new_pose = sfm.GetNewPose(	this->pose_actor,
														this->last_pose_actor,
														this->velocity_actual,
														this->target,
														sf,
														dt,
														0);

	if ( print_info ) {
		std::cout << "\t NEW pose: " << new_pose;
		std::cout << "\t\t distance to TARGET: " << (this->target - this->pose_actor.Pos()).Length() << std::endl;
		std::cout << std::endl << std::endl;
	}

	//
	double to_target_distance = to_target_vector.Length();

	// Choose a new target position if the actor has reached its current target.
	if (to_target_distance < 0.3) {
		if ( print_info ) {
			std::cout << "CHOOSE NEW TARGET! \t current: " << this->target;
		}
		this->chooseNewTarget();
		if ( print_info ) {
			std::cout << " \t new: " << this->target << std::endl;
		}
		to_target_vector = this->target - this->pose_actor.Pos();
	}

	// depending on the current stance SET the ROLL and PITCH angles properly
//	new_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, 1.5707 + new_pose.Rot().Euler().Z());

	// make sure the actor won't go out of bounds
	new_pose.Pos().X(std::max(-3.0, std::min(3.5, new_pose.Pos().X())));
	new_pose.Pos().Y(std::max(-10.0, std::min(2.0, new_pose.Pos().Y())));

	// depending on the current stance SET the Z coordinate properly
//	new_pose.Pos().Z(1.2138);

	// object info update
	double distanceTraveled = (new_pose.Pos() - this->actor->WorldPose().Pos()).Length();

	// update actor's world pose
	this->actor->SetWorldPose(new_pose, false, false);

	// update script time to set proper animation speed
	this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animation_factor));
	this->last_update = _info.simTime;

	print_info = false;
	Print_Set(false);

  	// save last position to calculate velocity
	last_pose_actor = this->actor->WorldPose();

	//// "pos"  was the vector from current location to target before
	//// ignition::math::Vector3d pos = this->target - pose.Pos();

	// ============================= GAZEBO DEFAULT ========================================= //

	// Normalize the direction vector, and apply the target weight
//	pos = pos.Normalize() * this->targetWeight;

	// Adjust the direction vector by avoiding obstacles
//	this->HandleObstacles(pos);

	// Compute the yaw orientation
//	ignition::math::Angle yaw = atan2(new_pose.Pos().Y(), new_pose.Pos().X()) + 1.5707 - rpy.Z();
//	yaw.Normalize();

	// Rotate in place, instead of jumping.
	// yaw_DELTA!
//	if (std::abs(yaw.Radian()) > IGN_DTOR(10)) {
//		new_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian()*0.001);
//	} else {
//		new_pose.Pos() += pos * this->velocity_desired * dt;
//		new_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z() + yaw.Radian());
//	}

	// Make sure the actor stays within bounds
//	new_pose.Pos().X(std::max(-3.0, std::min(3.5, new_pose.Pos().X())));
//	new_pose.Pos().Y(std::max(-10.0, std::min(2.0, new_pose.Pos().Y())));
//	new_pose.Pos().Z(1.2138);

	// Distance traveled is used to coordinate motion with the walking
	// animation
//	double distanceTraveled = (new_pose.Pos() - this->actor->WorldPose().Pos()).Length();

	// this->actor->SetWorldPose(pose, false, false);
//	this->actor->SetWorldPose(new_pose, false, false);

//	this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
//	this->lastUpdate = _info.simTime;

//	print_info = false;
//	Print_Set(false);


  	/*
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();



  	// -----------------------------------------------------------------------

    // when speed is 0 then there is no way of calculating the angle THETA_alpha_beta (0 vector length)
  	// better way will be calculating based on actor's yaw
    ignition::math::Angle yaw_inv;
    yaw_inv.Radian(rpy.Z());
    yaw_inv += yaw_inv.Pi;
    yaw_inv.Normalize();

    ignition::math::Vector3d n_alpha_experiment;
    n_alpha_experiment.X(sin(yaw_inv.Radian()));
    n_alpha_experiment.Y(cos(yaw_inv.Radian()));
    n_alpha_experiment.Z(0.0); // in-plane movement only now


  	bool vel_calc = false;
    vel_calc = CalculateVelocity(pose.Pos(), dt);
    this->actor->SetLinearVel(this->velocity_actual);
    this->world->ModelByName(this->actor->GetName())->SetLinearVel(this->velocity_actual);

	ignition::math::Vector3d n_alpha;
	ignition::math::Pose3d target_pose;
	target_pose.Set(this->target.X(), this->target.Y(), this->target.Z(), 0.0, 0.0, 0.0);

	// ********* debug
	static ignition::math::Pose3d pose_actor2;
	static ignition::math::Vector3d vel_actor2;

	if ( this->actor->GetName() == "actor2" ) {
		pose_actor2 = pose;
		vel_actor2 = this->velocity_actual;
		this->actor->SetLinearVel(this->velocity_actual);
	}
	// *********


	// static ignition::math::Pose3d new_pose = pose;

	ignition::math::Vector3d sf = sfm.GetSocialForce(this->world,
													 this->actor->GetName(),
													 pose,
													 this->velocity_actual,
													 target_pose.Pos());

	ignition::math::Pose3d new_pose = sfm.GetNewPose(pose, this->velocity_actual, sf, dt, 0);

	if ( (_info.simTime - print_time).Double() > 0.2 ) {

		if ( this->actor->GetName() == "actor1" ) {


			//if ( vel_calc ) {
			//	std::cout << " oooooo VELOCITY calculated oooooo " << " x: " << this->velocity_actual.X() << " y: " << this->velocity_actual.Y() << " z: " << this->velocity_actual.Z() << std::endl;
			//}

			//std::cout << std::setprecision(5) << "pose_cur: " << pose.Pos(); // << std::endl;
			//std::cout << std::setprecision(5) << "\t pose_last: " << last_pos_actor;
			//std::cout << std::setprecision(5) << "\t velocity: " << velocity_actual; // << std::endl;
			//std::cout << std::setprecision(5) << "\t dt: " << dt;
			//std::cout << std::endl;


			std::cout << "--------------------------------------------------------------------------------------------------------------" << std::endl;
			std::cout << " \t motion dir YAW: " << rpy.Z() << "\t motion dir YAW_'INV': " << yaw_inv;
			// sfm.GetInternalAcceleration(pose, this->velocity_actual, target_pose, &n_alpha, this->actor->GetName());
			//sfm.GetInternalAcceleration(pose, this->velocity_actual, target_pose);
			sfm.GetInternalAcceleration(pose, this->velocity_actual, target_pose.Pos());

			n_alpha = sfm.GetNormalToAlphaDirection(pose);
			std::cout << "\t AFTER  -- n_alpha: " << n_alpha << std::endl;


			//sfm.GetAngleBetweenObjectsVelocities(	pose, this->velocity_actual,
			//										ignition::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
			//										ignition::math::Vector3d(0.8, 0.8, 0.0) );

			ignition::math::Angle actor_yaw, object_yaw;

			//sfm.GetAngleBetweenObjectsVelocities(	pose, this->velocity_actual, &actor_yaw,
			//										pose_actor2, vel_actor2, &object_yaw );
//			sfm.GetAngleBetweenObjectsVelocities(pose, &actor_yaw, pose_actor2, &object_yaw);

//			ignition::math::Vector3d d_alpha_beta = pose_actor2.Pos() - pose.Pos();
//			uint8_t beta_rel_loc = sfm.GetBetaRelativeLocation(actor_yaw, d_alpha_beta);

//			if ( beta_rel_loc != 2) {
//				ignition::math::Vector3d p_alpha = sfm.GetPerpendicularToNormal(n_alpha, beta_rel_loc);
//			}

			std::cout << "======================  SOCIAL_FORCE_CALC  =======================" << std::endl;
			ignition::math::Vector3d sf = sfm.GetSocialForce(this->world,
															 this->actor->GetName(),
															 pose,
															 this->velocity_actual,
															 target_pose.Pos());
			std::cout << " TOTAL force: " << sf << std::endl;

			std::cout << "***********************  NEW_POSE_CALC  **************************" << std::endl;
			new_pose = sfm.GetNewPose(pose, this->velocity_actual, sf, dt, 0);
			std::cout << "\t NEW pose: " << new_pose << std::endl;
			std::cout << "\t distance to TARGET: " << pos.Length() << std::endl;
			std::cout << std::endl << std::endl;
			this->actor->SetWorldPose(new_pose, false, false);
			print_time = _info.simTime;


			std::cout << "======================  SOCIAL_FORCE_CALC  =======================" << std::endl;
			std::cout << " TOTAL force: " << sf << std::endl;

		}

	}

	// save the position of the actor to calculate velocity in the next step
	last_pos_actor = pose.Pos();
	return;


	// -----------------------------------------------------------------------

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
	  std::cout << "CHOOSE NEW TARGET! \t current: " << this->target;
    this->ChooseNewTarget();
    	std::cout << " \t new: " << this->target << std::endl;
    pos = this->target - pose.Pos();

  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity_desired * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  // this->actor->SetWorldPose(pose, false, false);
  this->actor->SetWorldPose(new_pose, false, false);

  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
  */

#endif

}

#ifndef REFACTOR_COMMON

void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

#endif

// ===============================================================================================

bool ActorPlugin::CalculateVelocity(const ignition::math::Vector3d &_pos, const double &_dt) {

//  	/// calculate actor's velocity vector or leave the last velocity
//	/// max allowable speed is 15 m/s
//	const unsigned int SPEED_LIMIT = 15;
//	double temp_x, temp_y, temp_z = 0.0;
//
//	// std::cout << " Calc vel! " << std::endl;
//	if ( (temp_x = (_pos.X() - this->last_pose_actor.Pos().X())/_dt) > SPEED_LIMIT ) {
//		if ( print_info ) {
//			std::cout << " oooooo VELOCITY SHOOTOUT X oooooo " << std::endl;
//		}
//		return false;
//	}
//
//	if ( (temp_y = (_pos.Y() - this->last_pose_actor.Pos().Y())/_dt) > SPEED_LIMIT ) {
//		if ( print_info ) {
//			std::cout << " oooooo VELOCITY SHOOTOUT Y oooooo " << std::endl;
//		}
//		return false;
//	}
//
//	if ( (temp_z = (_pos.Z() - this->last_pose_actor.Pos().Z())/_dt) > SPEED_LIMIT ) {
//		if ( print_info ) {
//			std::cout << " oooooo VELOCITY SHOOTOUT Z oooooo " << std::endl;
//		}
//		return false;
//	}
//
//	// if got there, then speed is in the permitted range - update the velocity
//	this->velocity_actual.X(temp_x);
//	this->velocity_actual.Y(temp_y);
//	this->velocity_actual.Z(temp_z);


	// TODO: RETURN bool DEPRECATED
	ignition::math::Vector3d new_velocity;
	new_velocity.X( (_pos.X() - this->last_pose_actor.Pos().X()) / _dt );
	new_velocity.Y( (_pos.Y() - this->last_pose_actor.Pos().Y()) / _dt );
	new_velocity.Z( (_pos.Z() - this->last_pose_actor.Pos().Z()) / _dt );

	/* Velocity Averaging Block -
	 * used there to prevent some kind of oscillations in
	 * social force algorithm execution - the main reason of such behavior were changes
	 * in speed which cause relative velocity fluctuations which on turn affects final
	 * result a lot */

	std::rotate( velocities_to_avg.begin(), velocities_to_avg.begin()+1, velocities_to_avg.end() );
	velocities_to_avg.at(velocities_to_avg.size()-1) = new_velocity;

	// sum up - at the moment no Z-velocities are taken into consideration
	double vel[3] = {0.0, 0.0, 0.0};
	for ( size_t i = 0; i < velocities_to_avg.size(); i++ ) {
		vel[0] += velocities_to_avg.at(i).X();
		vel[1] += velocities_to_avg.at(i).Y();
		// vel[2] += velocities_to_avg.at(i).Z();
	}

	vel[0] = vel[0] / static_cast<double>(velocities_to_avg.size());
	vel[1] = vel[1] / static_cast<double>(velocities_to_avg.size());
	// vel[2] = vel[2] / static_cast<double>(velocities_to_avg.size());

	new_velocity.X(vel[0]);
	new_velocity.Y(vel[1]);
	new_velocity.Z(vel[2]);

	this->velocity_actual = new_velocity;
	return true;

}

// ===============================================================================================

bool ActorPlugin::ReadSDF() {

	  // Read in the target weight
	  if (this->sdf ->HasElement("target_weight"))
	    this->targetWeight = this->sdf ->Get<double>("target_weight");
	  else
	    this->targetWeight = 1.15;

	  // Read in the obstacle weight
	  if (this->sdf ->HasElement("obstacle_weight"))
	    this->obstacleWeight = this->sdf ->Get<double>("obstacle_weight");
	  else
	    this->obstacleWeight = 1.5;

	  // Read in the animation factor (applied in the OnUpdate function).
	  if (this->sdf ->HasElement("animation_factor"))
	    this->animation_factor = this->sdf ->Get<double>("animation_factor");
	  else
	    this->animation_factor = 4.5;

	  // Add our own name to models we should ignore when avoiding obstacles.
	  this->ignoreModels.push_back(this->actor->GetName());

	  // Read in the other obstacles to ignore
	  if (this->sdf ->HasElement("ignore_obstacles"))
	  {
	    sdf::ElementPtr modelElem =
	    		this->sdf ->GetElement("ignore_obstacles")->GetElement("model");
	    while (modelElem)
	    {
	      this->ignoreModels.push_back(modelElem->Get<std::string>());
	      modelElem = modelElem->GetNextElement("model");
	    }
	  }

	  return true;

}

// ===============================================================================================

bool ActorPlugin::AlignToTargetDirection(ignition::math::Vector3d *_rpy) {

	// calculate the yaw angle actor need to rotate around world's Z axis
	// NOTE: +90 deg because actor's coordinate system is rotated 90 deg counter clockwise
	// V1 ------------------------------------------------------------------------------------------------------
//	ignition::math::Angle yaw_target(std::atan2( this->target.Y(), this->target.X() ) + (IGN_PI/2) );
	// V2 ------------------------------------------------------------------------------------------------------
	// yaw target expressed as an angle that depends on current IDEAL_TO_TARGET vector
	ignition::math::Vector3d to_target_vector = this->target - this->pose_actor.Pos();							// in world coord. system
	to_target_vector.Normalize();
	ignition::math::Angle yaw_target(std::atan2( to_target_vector.Y(), to_target_vector.X() ) + (IGN_PI/2) );	// +90 deg transforms the angle from actor's coord. system to the world's one
	//    ------------------------------------------------------------------------------------------------------
	yaw_target.Normalize();

	double yaw_start = this->pose_actor.Rot().Yaw();
	//ignition::math::Angle yaw_diff( yaw_start - yaw_target.Radian() );
	ignition::math::Angle yaw_diff( yaw_target.Radian() - yaw_start );
	yaw_diff.Normalize();

	/*
	// choose the right rotation direction (but the direction already calculated properly)
	if ( std::fabs(yaw_diff.Radian()) > (IGN_PI/2) ) {

		if ( yaw_diff.Radian() >= 1e-06 ) {
			// positive value
			yaw_diff.Radian( -(IGN_PI - yaw_diff.Radian() ));
		} else {
			// negative value
			yaw_diff.Radian( IGN_PI - std::fabs(yaw_diff.Radian() ));
		}

		if ( this->actor->GetName() == "actor1" ) {
			//if ( ctr == 0 ) {
				std::cout << "\tyaw_diff_changed: " << yaw_diff.Radian() << std::endl;
			//}
		}

	}
	*/

	// smooth the rotation if too big
	static const double YAW_INCREMENT = 0.001;
#define IVERT_SIGN // ok with that setting

#ifndef IVERT_SIGN
	short int sign = -1;
#else
	short int sign = +1;
#endif
//	ignition::math::Vector3d rpy = this->pose_actor.Rot().Euler();

	// check the sign of the diff - the movement should be performed in the OPPOSITE direction to yaw_diff angle
	if ( yaw_diff.Radian() < 0.0f ) {
#ifndef IVERT_SIGN
		sign = +1;
#else
		sign = -1;
#endif
	}

	// save the change to tell if actor is already aligned or not
	double angle_change = 0.0;

	// consider the difference (increment or decrement)
	if ( std::fabs(yaw_diff.Radian()) < YAW_INCREMENT ) {
		angle_change = static_cast<double>(sign) * yaw_diff.Radian();
	} else {
		angle_change = static_cast<double>(sign) * YAW_INCREMENT;
	}

	ignition::math::Angle yaw_result(yaw_start + angle_change);
	yaw_result.Normalize();
	_rpy->Z(yaw_result.Radian());

	// return true if the yaw_diff is small enough, otherwise return false
	yaw_diff.Radian(yaw_diff.Radian() - angle_change);
	yaw_diff.Normalize();

	if ( std::fabs(yaw_diff.Radian()) < IGN_DTOR(10) ) {
		return true;
	}
	return false;

}

// ===============================================================================================

inline void ActorPlugin::SetActorPose(const ignition::math::Pose3d &_pose) {

	this->pose_actor = _pose;

}

// ===============================================================================================

ignition::math::Vector3d ActorPlugin::UpdateActorOrientation(const ignition::math::Pose3d &_pose) {

	this->SetActorPose(_pose);
	return (this->UpdateActorOrientation());

}

// ===============================================================================================

ignition::math::Vector3d ActorPlugin::UpdateActorOrientation() {

	/* Corrects the rotation to align face with x axis if yaw = 0
	 * and stand up (with roll 0 actor is lying) */

	ignition::math::Vector3d rpy = this->pose_actor.Rot().Euler();

	switch (stance_actor) {

		// Yaw alignment with X-axis DEPRECATED //
		case(ACTOR_STANCE_WALK):
				rpy.X(1.5707);
//				rpy.Z() += IGN_DTOR(90);
				break;
		case(ACTOR_STANCE_STAND):
				rpy.X(1.5707);
//				rpy.Z() += IGN_DTOR(90);
				break;
		case(ACTOR_STANCE_LIE):
				rpy.X(0.0000);
//				rpy.Z() += IGN_DTOR(90);
				break;

	}

	return rpy;
}

// ===============================================================================================

#ifdef CREATE_ROS_NODE

void ActorPlugin::PublishActorTf() {

	geometry_msgs::TransformStamped msg;

	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = this->actor->GetName();

	msg.transform.translation.x = this->pose_actor.Pos().X();
	msg.transform.translation.y = this->pose_actor.Pos().Y();
	msg.transform.translation.z = 0.0f; // this->pose_actor.Pos().Z();

	msg.transform.rotation.x = this->pose_actor.Rot().X();
	msg.transform.rotation.y = this->pose_actor.Rot().Y();
	msg.transform.rotation.z = this->pose_actor.Rot().Z();
	msg.transform.rotation.w = this->pose_actor.Rot().W();

	tf_broadcaster.sendTransform(msg);

}

#endif

// ===============================================================================================

#ifdef VISUALIZE_SFM

void ActorPlugin::VisualizeForceField() {

#ifdef VIS_SFM_POINT

	ignition::math::Vector3d sf;

//	std::cout << "\n\n\n\nGET SOCIAL FORCE FOR VISUALIZATION " << this->actor->GetName();
	sf = sfm.GetSocialForce( this->world,
							 this->actor->GetName(),
							 this->pose_actor,
							 this->velocity_actual,
							 this->target,
							 this->actor_common_info);
//							 actor_common_info.getNameIDMap(),
//							 actor_common_info.getLinearVelocitiesVector(),
//							 actor_common_info.getBoundingBoxesVector());
//	std::cout << "\nGET SOCIAL FORCE FOR VISUALIZATION\n\t\tEND\n\n\n\n";


//	for ( size_t i = 0; i < sfm.GetClosestPointsVector().size(); i++ ) {
//		sfm_vis.SetPointArrow(sfm.GetClosestPointsVector()[i], i);
//	}


	// set closest to actor model points
	sfm_vis.SetColor(1.0, 1.0, 0.0, 0.5);
	for ( size_t i = 0; i < sfm.GetClosestPointsVector().size(); i=i+2 ) {
		sfm_vis.SetPointsLines(sfm.GetClosestPointsVector()[i], sfm.GetClosestPointsVector()[i+1], i);
	}

	// set general force for the actor
	sf_vis.SetColor(1.0, 0.0, 0.0, 1.0);
	sf_vis.SetForcePoint(	sf,
							ignition::math::Vector3d(this->pose_actor.Pos().X(), this->pose_actor.Pos().Y(), 0.0f),
							actor_common_info.GetActorID());


#ifdef CREATE_ROS_NODE
	vis_pub.publish(sfm_vis.GetMarkerArray());
#elif defined(CREATE_ROS_INTERFACE)
	ros_interface.PublishMarkerArray(sfm_vis.GetMarkerArray()); // closest to actor model points
	ros_interface.PublishSFArrows(   sf_vis. GetMarkerArray()); // result force for the actor
#endif

#elif defined(VIS_SFM_GRID)

	//sfm_vis.createGrid(-3.0, 3.5, -10.0, 2.0, 1.0);
	sfm_vis.CreateGrid(-5.0, 5.5, -12.0, 4.0, 0.75);

	ignition::math::Pose3d pose;
	ignition::math::Vector3d sf;

//	size_t iter = 0;
//	std::cout << "sfm_vis:" << iter << std::endl;

	while ( !sfm_vis.IsWholeGridChecked() ) {

		pose = ignition::math::Pose3d(sfm_vis.GetNextGridElement(), this->pose_actor.Rot());


		/*
		 * Remember to artificially place the actor (along with his bounding) in current grid cell!
		 */
#if	defined(INFLATE_BOUNDING_BOX)
		actor_common_info.SetBoundingBox( this->GenerateBoundingBox(pose) );
#elif defined(INFLATE_BOUNDING_CIRCLE)
		this->bounding_circle.SetCenter(pose.Pos());
		this->actor_common_info.SetBoundingCircle(this->bounding_circle);
#elif defined(INFLATE_BOUNDING_ELLIPSE)

#endif


		sf = sfm.GetSocialForce( this->world,
								 this->actor->GetName(),
								 pose,
								 this->velocity_actual,
								 this->target,
								 this->actor_common_info);
		sfm_vis.SetForce(sf);

//		std::cout << "sfm_vis:" << iter << "\tsf: " << sf << std::endl;
//		iter++;

	}

#ifdef CREATE_ROS_NODE
	vis_pub.publish(sfm_vis.GetMarkerArray());
#elif defined(CREATE_ROS_INTERFACE)
	ros_interface.PublishMarkerArray(sfm_vis.GetMarkerArray());
#endif

	// std::cout << sfm_vis.getMarkerArray().markers << std::endl;
	// sfm_vis.publishMarkerArray();

	sfm_vis.ResetGridIndex();

#endif

/* */
	// calculate grid for actor1

	if ( this->actor->GetName() == "actor1" ) {

		grid_vis.CreateGrid(-5.0, 5.5, -12.0, 4.0, 0.75);

		ignition::math::Pose3d pose;
		ignition::math::Vector3d sf;

		while ( !grid_vis.IsWholeGridChecked() ) {

			pose = ignition::math::Pose3d(grid_vis.GetNextGridElement(), this->pose_actor.Rot());

			// Remember to artificially place the actor (along with his bounding) in current grid cell!
	#if	defined(INFLATE_BOUNDING_BOX)
			this->actor_common_info.SetBoundingBox( this->GenerateBoundingBox(pose) );
	#elif defined(INFLATE_BOUNDING_CIRCLE)
			this->bounding_circle.SetCenter(pose.Pos());
			this->actor_common_info.SetBoundingCircle(this->bounding_circle);
	#elif defined(INFLATE_BOUNDING_ELLIPSE)

	#endif

			sf = sfm.GetSocialForce( this->world,
									 this->actor->GetName(),
									 pose,
									 this->velocity_actual,
									 this->target,
									 this->actor_common_info);
			grid_vis.SetForce(sf);

		}

		#if defined(CREATE_ROS_INTERFACE)
		ros_interface.PublishMarkerArrayGrid(grid_vis.GetMarkerArray());
		#endif

		grid_vis.ResetGridIndex();

	}


}

#endif

// ===============================================================================================

double ActorPlugin::PrepareForUpdate(const common::UpdateInfo &_info) {

	this->SetActorPose(this->actor->WorldPose());

//	ignition::math::Vector3d rpy = this->UpdateActorOrientation();
//	this->pose_actor.Rot().Euler().X(rpy.X());
//	this->pose_actor.Rot().Euler().Y(rpy.Y());
//	this->pose_actor.Rot().Euler().Z(rpy.Z());

	this->pose_actor.Rot().Euler(this->UpdateActorOrientation());

	double dt = (_info.simTime - this->last_update).Double();
	CalculateVelocity(this->pose_actor.Pos(), dt);

	actor_common_info.SetLinearVel(this->velocity_actual);

	this->actor->SetLinearVel(this->velocity_actual);

	/* update the bounding box/circle/ellipse of the actor
	 * (aim is to create a kind of inflation layer) */

#if	defined(INFLATE_BOUNDING_BOX)
	actor_common_info.SetBoundingBox( this->GenerateBoundingBox(this->pose_actor) );
#elif defined(INFLATE_BOUNDING_CIRCLE)
	bounding_circle.SetCenter(this->pose_actor.Pos());
	actor_common_info.SetBoundingCircle(bounding_circle);
#elif defined(INFLATE_BOUNDING_ELLIPSE)

#endif

	// dt is helpful for further calculations
	return dt;

}

// ===============================================================================================

void ActorPlugin::ApplyUpdate(const common::UpdateInfo &_info, const double &_dist_traveled) {

  	// save last position to calculate velocity
	this->last_pose_actor = this->actor->WorldPose();

	// update the global pose
	this->actor->SetWorldPose(this->pose_actor, false, false);

	/*
	 * std::cout << this->actor->GetName() << " | script time: " << this->actor->ScriptTime() << "\tdist_trav: " << _dist_traveled << "\tanim_factor: " << this->animation_factor << std::endl;
	 *
	 * for some reason (very likely some very small number returned as interaction force
	 * in social force model) dist_traveled sometimes turns out to be a NaN - then change
	 * it to a typical value of 0.005; NaN value of dist seems to be the cause of such error:
	 *
	 * "gazebo::common::NodeAnimation::FrameAt(double, bool) const: Assertion
	 *  `(t >= 0.0 && t <= 1.0)&&("t is not in the range 0.0..1.0")' failed."
	 *
	 *  FIXME: if such error occurs uncomment below line (not done for debugging process)
	 *  (std::isnan(_dist_traveled)) ? (_dist_traveled = 0.0005) : (0);
	 *
	 */

	// update script time to set proper animation speed
	this->actor->SetScriptTime(this->actor->ScriptTime() + (_dist_traveled * this->animation_factor));

	// update time
	this->last_update = _info.simTime;

	// debug info
	print_info = false;
	Print_Set(false);

}

// ===============================================================================================

void ActorPlugin::ActorStateAlignTargetHandler(const common::UpdateInfo &_info) {

	this->PrepareForUpdate(_info);

	// copy the actor's current rotation to local variable
	ignition::math::Vector3d new_rpy = this->pose_actor.Rot().Euler();

	/* if already aligned - switch to a certain state, otherwise proceed to the next
	 * rotation procedure */
	if ( this->AlignToTargetDirection(&new_rpy) ) {
#ifndef SILENT_
		std::cout << "\n\n\t\t\tALIGNED\n\n";
#endif
		state_actor = ACTOR_STATE_MOVE_AROUND;
	}

	// update the local copy of the actor's pose
//	this->pose_actor.Set(this->pose_actor.Pos(), new_rpy);
	this->SetActorPose(ignition::math::Pose3d(this->pose_actor.Pos(), ignition::math::Quaterniond(new_rpy)));

	/* forced close-to-zero distance traveled to avoid actor oscillations;
	 * of course 0.0 linear distance is traveled when pure rotation is performed */
	this->ApplyUpdate(_info, 0.0007);

}

// ===============================================================================================

void ActorPlugin::ActorStateMoveAroundHandler(const common::UpdateInfo &_info) {

	// Social Force Model
	double dt = this->PrepareForUpdate(_info);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	ignition::math::Vector3d sf = sfm.GetSocialForce(this->world,
													 this->actor->GetName(),
													 this->pose_actor,
													 this->velocity_actual,
													 this->target,
													 this->actor_common_info);
//													 actor_common_info.getNameIDMap(),
//													 actor_common_info.getLinearVelocitiesVector(),
//													 actor_common_info.getBoundingBoxesVector());

	if ( print_info ) {
		std::cout << "\t TOTAL force: " << sf << std::endl;
		std::cout << "\t lin_vels_vector: ";
		for ( int i = 0; i < actor_common_info.GetLinearVelocitiesVector().size(); i++ ) {
			std::cout << "\t" << actor_common_info.GetLinearVelocitiesVector()[i];
		}
		std::cout << std::endl;
		std::cout << "***********************  NEW_POSE_CALC  **************************" << std::endl;
	}

	ignition::math::Pose3d new_pose = sfm.GetNewPose(	this->pose_actor,
														this->last_pose_actor,
														this->velocity_actual,
														this->target,
														sf,
														dt,
														0);

	if ( print_info ) {
		std::cout << "\t NEW pose: " << new_pose;
		std::cout << "\t\t distance to TARGET: " << (this->target - this->pose_actor.Pos()).Length() << std::endl;
		std::cout << std::endl << std::endl;
	}

	// calculate a distance to a target
	double to_target_distance = (this->target - this->pose_actor.Pos()).Length();

	// choose a new target position if the actor has reached its current target
//	if (to_target_distance < 0.3) {

	/* the smaller tolerance the bigger probability that actor will
	 * step into some obstacle */
	if (to_target_distance < 1.25) {

		this->chooseNewTarget(_info);
		// after setting new target, first let's rotate to its direction
		state_actor = ACTOR_STATE_ALIGN_TARGET;

	}

	// make sure the actor won't go out of bounds
	// TODO: YAML config
//	new_pose.Pos().X( std::max(-3.0,  std::min( 3.5, new_pose.Pos().X() ) ) );
//	new_pose.Pos().Y( std::max(-10.0, std::min( 2.0, new_pose.Pos().Y() ) ) );

	// object info update
	double dist_traveled = (new_pose.Pos() - this->actor->WorldPose().Pos()).Length();

	// update the local copy of the actor's pose
	this->SetActorPose(new_pose);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	this->ApplyUpdate(_info, dist_traveled);

}

// ===============================================================================================

void ActorPlugin::ActorStateFollowObjectHandler(const common::UpdateInfo &_info) {

}

// ===============================================================================================

void ActorPlugin::ActorStateTeleoperationHandler(const common::UpdateInfo &_info) {

}

// ===============================================================================================
