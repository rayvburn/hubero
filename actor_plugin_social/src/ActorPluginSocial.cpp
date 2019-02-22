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
std::vector<ignition::math::Vector3d> lin_vels_vector;
std::map<std::string, unsigned int> map_of_names;



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
    std::cout << "LOADED POSE: " << this->actor->WorldPose() << std::endl;

  	ignition::math::Vector3d init_orient = this->actor->WorldPose().Rot().Euler();
  	ignition::math::Pose3d init_pose;
  	ignition::math::Vector3d init_position;
  	init_position = this->actor->WorldPose().Pos();
  	init_position.Z(1.2138);

  	init_pose.Set(init_position,
				  ignition::math::Quaterniond(init_orient.X() + IGN_PI/2,
											  init_orient.Y(),
											  init_orient.Z() + 1.5707));
	this->actor->SetWorldPose(init_pose, false, false);
	std::cout << " -------- SET WorldPose() actor! -------- " << init_pose << std::endl;

	// conversions between Euler and Quaternion will finally produce the result that converges to 0...
	// above is deprecated, yaw will be set in each OnUpdate()

	// Set last_pos_actor to prevent velocity shootout
	last_pose_actor.Pos() = this->actor->WorldPose().Pos();
	std::cout << " -------- SET last_pos_actor! -------- " << last_pose_actor.Pos() << std::endl;

	actor_id = this->InitActorInfo(this->actor->GetName());
	std::cout << " -------- ACTOR ID -------- " << actor_id << std::endl;

	// WARNING: HARD-CODED target coord
	if ( this->actor->GetName() == "actor1" ) {
		this->target.Y(-3.50);
	}

}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity_desired = 0.8;
  this->lastUpdate = 0;

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
void ActorPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////

unsigned int ActorPlugin::InitActorInfo(const std::string &_name) {

//	ignition::math::Vector3d vel(0.0, 0.0, 0.0);

	// to check if the vector is shared between objects
//    std::cout << "InitActorInfo()   lin vels address" << &lin_vels_vector << std::endl;
//	lin_vels_vector.push_back(vel);
	lin_vels_vector.emplace_back(1.0, 1.0, 0.0);
	std::cout << "\tlin vels vector[ ]: " << lin_vels_vector[lin_vels_vector.size()-1] << std::endl;

	unsigned int id = static_cast<unsigned int>(lin_vels_vector.size() - 1);
	map_of_names.insert(std::make_pair(_name, id));
	return id;
}

/////////////////////////////////////////////////

void ActorPlugin::SetActorsLinearVel(const unsigned int &_id, const ignition::math::Vector3d &_vel) {
//	lin_vels_vector[_id] = _vel;
	lin_vels_vector.at(_id) = _vel;
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{

	// debugging purposes

  	static common::Time print_time;

  	if ( (_info.simTime - print_time).Double() > 0.2 ) {
		if ( this->actor->GetName() == "actor1" ) {
			print_info = true;
			Print_Set(true);
			print_time = _info.simTime;
			std::cout << "**************************************************** ACTOR1 ***********************************************" << std::endl;
			std::cout << "**** INITIAL pose: " << this->actor->WorldPose() << "\t\t actor1 velocity: \t" << this->velocity_actual << "\t target: " << this->target << std::endl;
		}
	} else {
		print_info = false;
	}

  	// OnUpdate algorithm
  	double dt = (_info.simTime - this->lastUpdate).Double();
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
													 lin_vels_vector);
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
		this->ChooseNewTarget();
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
	this->actor->SetScriptTime(this->actor->ScriptTime() + (distanceTraveled * this->animationFactor));
	this->lastUpdate = _info.simTime;

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
}

// ===============================================================================================

bool ActorPlugin::CalculateVelocity(const ignition::math::Vector3d &_pos, const double &_dt) {

  	/// calculate actor's velocity vector or leave the last velocity
	/// max allowable speed is 15 m/s
	const unsigned int SPEED_LIMIT = 15;
	double temp_x, temp_y, temp_z = 0.0;

	// std::cout << " Calc vel! " << std::endl;
	if ( (temp_x = (_pos.X() - this->last_pose_actor.Pos().X())/_dt) > SPEED_LIMIT ) {
		if ( print_info ) {
			std::cout << " oooooo VELOCITY SHOOTOUT X oooooo " << std::endl;
		}
		return false;
	}

	if ( (temp_y = (_pos.Y() - this->last_pose_actor.Pos().Y())/_dt) > SPEED_LIMIT ) {
		if ( print_info ) {
			std::cout << " oooooo VELOCITY SHOOTOUT Y oooooo " << std::endl;
		}
		return false;
	}

	if ( (temp_z = (_pos.Z() - this->last_pose_actor.Pos().Z())/_dt) > SPEED_LIMIT ) {
		if ( print_info ) {
			std::cout << " oooooo VELOCITY SHOOTOUT Z oooooo " << std::endl;
		}
		return false;
	}

	// if got there, then speed is in the permitted range - update the velocity
	this->velocity_actual.X(temp_x);
	this->velocity_actual.Y(temp_y);
	this->velocity_actual.Z(temp_z);
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
	    this->animationFactor = this->sdf ->Get<double>("animation_factor");
	  else
	    this->animationFactor = 4.5;

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

//inline void ActorPlugin::RestoreYawGazeboInPose() {
//	this->pose_actor.Rot().Euler().Z() -= IGN_DTOR(90);
//}
