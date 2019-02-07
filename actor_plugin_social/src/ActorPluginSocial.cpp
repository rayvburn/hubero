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






using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	std::cout << "ACTOR PLUGIN::LOAD!" << std::endl;
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

    // Set last_pos_actor to prevent velocity shootout
	last_pos_actor = this->actor->WorldPose().Pos();
	std::cout << " -------- SET POSE! -------- " << last_pos_actor << std::endl;

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
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{

  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

	// modelcount
	// model by index
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

	ignition::math::Vector3d n_alpha;
	ignition::math::Pose3d target_pose;
	target_pose.Set(this->target.X(), this->target.Y(), this->target.Z(), 0.0, 0.0, 0.0);

	// ********* debug
	static ignition::math::Pose3d pose_actor2;
	static ignition::math::Vector3d vel_actor2;

	if ( this->actor->GetName() == "actor2" ) {
		pose_actor2 = pose;
		vel_actor2 = this->velocity_actual;
	}
	// *********

	static common::Time print_time;
	if ( (_info.simTime - print_time).Double() > 0.2 ) {

		if ( this->actor->GetName() == "actor1" ) {

			/*
			if ( vel_calc ) {
				std::cout << " oooooo VELOCITY calculated oooooo " << " x: " << this->velocity_actual.X() << " y: " << this->velocity_actual.Y() << " z: " << this->velocity_actual.Z() << std::endl;
			}

			std::cout << std::setprecision(5) << "pose_cur: " << pose.Pos(); // << std::endl;
			std::cout << std::setprecision(5) << "\t pose_last: " << last_pos_actor;
			std::cout << std::setprecision(5) << "\t velocity: " << velocity_actual; // << std::endl;
			std::cout << std::setprecision(5) << "\t dt: " << dt;
			std::cout << std::endl;
			*/

			std::cout << " \t\t\t\t\t\t\t\t\t motion dir yaw: " << rpy.Z() << " motion dir yaw_inv: " << yaw_inv << std::endl;
			// sfm.GetInternalAcceleration(pose, this->velocity_actual, target_pose, &n_alpha, this->actor->GetName());
			sfm.GetInternalAcceleration(pose, this->velocity_actual, target_pose);

			n_alpha = sfm.GetNormalToAlphaDirection(pose);
			std::cout << "AFTER  -- n_alpha: " << n_alpha << std::endl;

			/*
			sfm.GetAngleBetweenObjectsVelocities(	pose, this->velocity_actual,
													ignition::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
													ignition::math::Vector3d(0.8, 0.8, 0.0) );
			*/
			ignition::math::Angle actor_yaw, object_yaw;

			sfm.GetAngleBetweenObjectsVelocities(	pose, this->velocity_actual, &actor_yaw,
													pose_actor2, vel_actor2, &object_yaw );

			ignition::math::Vector3d d_alpha_beta = pose_actor2.Pos() - pose.Pos();
			uint8_t beta_rel_loc = sfm.GetBetaRelativeLocation(actor_yaw, d_alpha_beta);

			if ( beta_rel_loc != 2) {
				ignition::math::Vector3d p_alpha = sfm.GetPerpendicularToNormal(n_alpha, beta_rel_loc);
			}

			print_time = _info.simTime;

		}

	}

	// save the position of the actor to calculate velocity in the next step
	last_pos_actor = pose.Pos();


	// -----------------------------------------------------------------------

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
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

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

bool ActorPlugin::CalculateVelocity(const ignition::math::Vector3d &_pos, const double &_dt) {

  	/// calculate actor's velocity vector or leave the last velocity
	/// max allowable speed is 15 m/s
	const unsigned int SPEED_LIMIT = 15;
	double temp_x, temp_y, temp_z = 0.0;

	if ( (temp_x = (_pos.X() - this->last_pos_actor.X())/_dt) > SPEED_LIMIT ) {
		std::cout << " oooooo VELOCITY SHOOTOUT X oooooo " << std::endl;
		return false;
	}

	if ( (temp_y = (_pos.Y() - this->last_pos_actor.Y())/_dt) > SPEED_LIMIT ) {
		std::cout << " oooooo VELOCITY SHOOTOUT Y oooooo " << std::endl;
		return false;
	}

	if ( (temp_z = (_pos.Z() - this->last_pos_actor.Z())/_dt) > SPEED_LIMIT ) {
		std::cout << " oooooo VELOCITY SHOOTOUT Z oooooo " << std::endl;
		return false;
	}

	// if got there, then speed is in the permitted range - update the velocity
	this->velocity_actual.X(temp_x);
	this->velocity_actual.Y(temp_y);
	this->velocity_actual.Z(temp_z);
	return true;

}
