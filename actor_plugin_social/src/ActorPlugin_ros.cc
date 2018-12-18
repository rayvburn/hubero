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
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cassert>
#include <cmath>
#include <ctime>
#include <chrono>

#include <ros/console.h>
#include <iostream>
#include <thread>
#include <string>
#include <ros/console.h> //roslogging
#include "ActorPlugin.hh"

#include <SocialForceModel.h>
#include <SocialForceModelUtils.h>

/////////////////////////////////////////////////////////////////////

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define ACTOR_ANIMATION "walking"

/////////////////////////////////////////////////////////////////////

ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////////////////////////

void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
	this->world = this->actor->GetWorld();
	this->start_location = this->actor->WorldPose().Pos();
	this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

	/*
	// TODO: select friends among present actors (social force, attraction factor)
	P(friend_present) = 0.15

	for ( actorIteration() ) {
		random_number = rand(0,1);
		if ( random_number < P(friend_present) ) {
			friends_vector.push_back( current_actor->getName() )
		}
	}

	*/

	// set initial speed from above
	this->velocity = 1.0;

	// Load the first target location
	if (_sdf->HasElement("target") )
		this->target = _sdf->Get<ignition::math::Vector3d>("target");
	else
		// set manually if there is no target
		this->target = ignition::math::Vector3d(0, -5, 1.2138);


	// Load the max speed parameter
	if (_sdf->HasElement("max_speed"))
		this->v_max = _sdf->Get<double>("max_speed");
	else
		// just take the average speed if not given
		this->v_max = 1.2;

	// TODO: extend this here or in OnUpdate()?
	// Add our own name to models we should ignore when avoiding obstacles.
	this->ignored_models.push_back(this->actor->GetName());

  	// Read in the other obstacles to ignore
	if (_sdf->HasElement("ignore_obstacles")) {

		sdf::ElementPtr modelElem = _sdf->GetElement("ignore_obstacles")->GetElement("model");
		while (modelElem) {
			this->ignored_models.push_back(modelElem->Get< std::string >());
			modelElem = modelElem->GetNextElement("model");
		}

	}

	auto skelAnims = this->actor->SkeletonAnimations();
	if ( skelAnims.find(ACTOR_ANIMATION) == skelAnims.end() ) {

		gzerr << "Skeleton animation " << ACTOR_ANIMATION << " not found.\n";

	} else {

		// Create custom trajectory
		this->trajectory_info.reset(new physics::TrajectoryInfo());
		this->trajectory_info->type = ACTOR_ANIMATION;
		this->trajectory_info->duration = 1.0;
		this->actor->SetCustomTrajectory(this->trajectory_info);

	}

	// Initialize ros, if it has not already been initialized
	if (!ros::isInitialized() ) {

		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);

	}

	this->ros_nh_ptr.reset(new ros::NodeHandle("gazebo_client"));

	getROSParameters(this->ros_nh_ptr);

	this->ros_nh_ptr->setCallbackQueue(&this->ros_cb_queue);

	// start running services
	this->set_pose_srv 		= this->ros_nh_ptr->advertiseService("/" + this->actor->GetName() + "/SetActorPosition", &ActorPlugin::SetPoseCallback, this);
	this->set_target_srv 		= this->ros_nh_ptr->advertiseService("/" + this->actor->GetName() + "/SetActorTarget", &ActorPlugin::SetTargetCallback, this);
	this->get_vel_srv 		= this->ros_nh_ptr->advertiseService("/" + this->actor->GetName() + "/GetActorVelocity", &ActorPlugin::GetVelCallback, this);

	this->vel_publisher 		= this->ros_nh_ptr->advertise<geometry_msgs::Twist>("/"+this->actor->GetName()+"/actor_vel",1);
	this->ros_queue_thread 	= std::thread(std::bind(&ActorPlugin::QueueThread, this));

}

/////////////////////////////////////////////////////////////////////

ignition::math::Vector3d ActorPlugin::CallActorVelClient(std::string actor_name_) const {

	ros::ServiceClient get_vel_client = this->ros_nh_ptr->serviceClient<actor_sim_srv::GetVel>("/"+actor_name_+"/GetActorVelocity");
	actor_sim_srv::GetVel getvel_srv;
	getvel_srv.request.set_flag = false;
	get_vel_client.call(getvel_srv);
	get_vel_client.shutdown();

	return ignition::math::Vector3d(getvel_srv.response.x, getvel_srv.response.y, 0);

}

/////////////////////////////////////////////////////////////////////
// Compute social force on the actor.
ignition::math::Vector3d ActorPlugin::SocialForce(ignition::math::Pose3d &_pose, ignition::math::Vector3d _velocity) const
{


	ignition::math::Vector3d force;

	for(unsigned int i = 0; i < this->world->ModelCount(); i++) {

		physics::ModelPtr current_agent = this->world->ModelByIndex(i);

		/* TURTLEBOT3 DEPRECATED HERE
		if ((!current_agent->HasType(physics::Base::EntityType::ACTOR)) &&
			(current_agent->GetName()!=this->tb3_name)) {

			continue;
		}

		if ( (!tb3_as_actor) && (current_agent->GetName()==this->tb3_name) ) {
			continue;
		}
		*/
		
		if (current_agent == this->actor) {
			continue;
		}

		ignition::math::Vector3d currentPose = current_agent->WorldPose().Pos();
		double distance = currentPose.Distance(_pose.Pos());

		if (distance > neighbor_range) {
			continue;
		}

		ignition::math::Vector3d diff = currentPose - _pose.Pos();
		ignition::math::Vector3d diffDirection = diff.Normalize();

		double otherAngle = atan2(diffDirection.Y(), diffDirection.X());
		ignition::math::Angle angle_fov(otherAngle+0.5*M_PI-_pose.Rot().Yaw());
		angle_fov.Normalize();

		if ( std::fabs(angle_fov.Radian()) > 0.5*depth_fov) {
			continue;
		}

		ignition::math::Vector3d other_vel = CallActorVelClient(current_agent->GetName());
		ignition::math::Vector3d velDiff = _velocity - other_vel;

		//if (this->actor->GetName()=="actor0"){
		//ROS_ERROR("%s, vel x: %lf, vel y: %lf", this->actor->GetName().c_str(), _velocity.X(), _velocity.Y());
		////ROS_ERROR("%s, angle fov: %lf", this->actor->GetName().c_str(), angle_fov.Radian());
		////ROS_ERROR("%s, vel x: %lf, vel y: %lf", currentAgent->GetName().c_str(), other_vel.X(), other_vel.Y());
		//}

		ignition::math::Vector3d interactionVector = sf_lambda_importance * velDiff + diffDirection;
		double interactionLength = interactionVector.Length();
		ignition::math::Vector3d interactionDirection = interactionVector / interactionLength;

		double thisAngle = atan2(interactionDirection.Y(), interactionDirection.X());
		ignition::math::Angle theta_angle(otherAngle-thisAngle);
		theta_angle.Normalize();
		double theta = theta_angle.Radian();

		// Get sign of theta.
		double thetaSign = (theta == 0.00) ? (0.0) : (theta / std::fabs(theta));

		// compute model parameter B = gamma * ||D||
		double B = sf_gamma * interactionLength;

		double forceVelocityAmount = -std::exp(-diff.Length()/B - (sf_n_prime * B * theta) * (sf_n_prime * B * theta));
		double forceAngleAmount = -thetaSign * std::exp(-diff.Length() / B - (sf_n * B * theta) * (sf_n * B * theta));

		ignition::math::Vector3d forceVelocity = forceVelocityAmount * interactionDirection;

		// Dodge to the direction preset
		ignition::math::Vector3d interactionDirectionNormal;
		if (this->dodging_right) {
			interactionDirectionNormal = ignition::math::Vector3d(-interactionDirection.Y(), interactionDirection.X(), interactionDirection.Z());
		} else {
			interactionDirectionNormal = ignition::math::Vector3d(interactionDirection.Y(), -interactionDirection.X(), interactionDirection.Z());
		}

		ignition::math::Vector3d forceAngle = forceAngleAmount * interactionDirectionNormal;
		force += forceVelocity + forceAngle;

	}

	//ROS_ERROR("======force: %lf======", force.X());
	return force;

}

/////////////////////////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{

	// TODO: algorithm as follows:
	/*
	if ( targetReached() ) {

		// select next animation and behavior
		if ( distanceToChair < threshold ) {
			set animation to sit down
			sit_down
			wait for 20 sec
			stand up
		} else if ( distanceToPerson < threshold && persons_speed < threshold_speed ) {
			try talking
			// consider attractiveness decrease with the time
		} else if ( distanceToCouch < threshold ) {
			lay down
			wait for 30 sec
			stand up
		} else if ( no_motion_for_5_sec ) {
			// seems like stuck
			choose new target
		} else {
			// meaning that there is no OBJECTS OF INTEREST nearby
			choose new target
		}

	}
	*/


	// Time delta
	double dt = (_info.simTime - this->last_update).Double();

	// Position of this actor
	ignition::math::Pose3d pose = this->actor->WorldPose();

	// Direct vector to the current target
	ignition::math::Vector3d pos = this->target - pose.Pos();
	ignition::math::Vector3d rpy = pose.Rot().Euler();

	// Get the desired force to waypoint: "I want to go there at full speed!"
	ignition::math::Vector3d desiredForce = pos.Normalize() * this->v_max;	// desired acceleration term
	// @CITATION relaxation times we used τ α = 0.5s. Smaller values of τ α

	ignition::math::Vector3d socialForce_ = SocialForce(pose, this->velocity);
	//ignition::math::Vector3d obstacleForce = ObstacleForce(pose);

	//std::chrono::time_point<std::chrono::system_clock> start, end;
	//start = std::chrono::system_clock::now();
	//end = std::chrono::system_clock::now();
	//std::chrono::duration<double> elapsed_seconds = end-start;
	//ROS_ERROR("time count: %lf", elapsed_seconds.count());
	//ignition::math::Vector3d a = (this->socialForceFactor * socialForce_) + (this->desiredForceFactor * desiredForce) + (this->obstacleForceFactor * obstacleForce);
	ignition::math::Vector3d a = (this->social_force_factor * socialForce_) + (this->desired_force_factor * desiredForce);

	//this->velocity = 0.5 * this->velocity + a * dt;
	this->velocity = this->velocity*this->vel_param + a * dt;

	double speed = this->velocity.Length();
	if (speed > this->v_max) {
		this->velocity = this->velocity.Normalize() * this->v_max;
	}

	ignition::math::Angle yaw_update = atan2(this->velocity.Y(), this->velocity.X()) + 0.5*M_PI - rpy.Z();
	yaw_update.Normalize();
  
	double temp_vel = this->velocity.Length();
  
	// Rotate in place, instead of jumping.
	if (std::fabs(yaw_update.Radian()) > (this->max_angle_update/180.0 * M_PI)){
		double yaw_update_sign = yaw_update.Radian()/std::fabs(yaw_update.Radian());
		yaw_update = this->max_angle_update/180 * M_PI*yaw_update_sign;
	}

	ignition::math::Angle new_yaw = rpy.Z()+yaw_update.Radian()-0.5*M_PI;
	this->velocity.X() = temp_vel * cos(yaw_update.Radian()) * cos(new_yaw.Radian());
	this->velocity.Y() = temp_vel * cos(yaw_update.Radian()) * sin(new_yaw.Radian());

	pose.Rot() = ignition::math::Quaterniond(0.5*M_PI, 0, new_yaw.Radian()+0.5*M_PI);
	yaw_vel = yaw_update.Radian()/dt;

	//ROS_ERROR("%s, yaw: %lf", this->actor->GetName().c_str(), rpy.Z()+yaw.Radian()-0.5*M_PI);
	pose.Pos() = pose.Pos() + this->velocity * dt;
	pose.Pos().Z(this->fixed_actor_height);

	double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

	PublishActorInfo(a, pos.Normalize(), socialForce_, new_yaw.Radian());
	this->actor->SetWorldPose(pose, false, false);
	this->actor->SetScriptTime( this->actor->ScriptTime() + (distanceTraveled * this->animation_factor) );
	this->last_update = _info.simTime;

	// ros stuff
	//static tf::TransformBroadcaster br;
	//tf::Transform tf_transform;
	//tf_transform.setOrigin(tf::Vector3(pose.Pos().X(),pose.Pos().Y(),0));
	//tf::Quaternion tf_q;
	//tf_q.setRPY(rpy.X()-0.5*M_PI, rpy.Y(), new_yaw.Radian());
	//tf_transform.setRotation(tf_q);
	//br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "default_world", this->actor->GetName()));
	//ros::spinOnce();

	double distance = pose.Pos().Distance(this->target);

	if (distance < sf_distance_threshold) {

		ignition::math::Vector3d temp = this->start_location;
		this->start_location = this->target;
		this->target = temp;
		pos = this->target - pose.Pos();

	}
  
}

/////////////////////////////////////////////////////////////////////
// Set target position service callback. Response is the target position right now
bool ActorPlugin::SetTargetCallback(actor_sim_srv::SetPose::Request& req, actor_sim_srv::SetPose::Response& res) {

	res.x = this->target.X();
	res.y = this->target.Y();

	if (req.set_flag == true) {

		ignition::math::Vector3d newTarget(this->target);
		newTarget.X(req.new_x);
		newTarget.Y(req.new_y);
		this->target = newTarget;

	}

	return true;

}

/////////////////////////////////////////////////////////////////////
bool ActorPlugin::GetVelCallback(actor_sim_srv::GetVel::Request& req, actor_sim_srv::GetVel::Response& res) {

	res.x = this->velocity.X();
	res.y = this->velocity.Y();
	res.yaw = this->yaw_vel;

	if (req.set_flag == true) {

		this->velocity.X(req.new_x);
		this->velocity.Y(req.new_y);
		this->yaw_vel = req.new_yaw;

	}

	return true;

}

/////////////////////////////////////////////////////////////////////
// \Set actor position service callback. Response is the position right now
bool ActorPlugin::SetPoseCallback(actor_sim_srv::SetPose::Request& req, actor_sim_srv::SetPose::Response& res) {

	ignition::math::Pose3d pose = this->actor->WorldPose();
	ignition::math::Vector3d pos = pose.Pos();
	res.x = pos.X();
	res.y = pos.Y();

	if (req.set_flag == true) {
		pose.Pos().X(req.new_x);
		pose.Pos().Y(req.new_y);
		this->actor->SetWorldPose(pose);
	}

	return true;
}

/////////////////////////////////////////////////////////////////////
/// \brief ROS helper function that processes messages
void ActorPlugin::QueueThread() {

  // It gonna be really slow if you change it to 0
  static const double timeout = 0.01;

  while (this->ros_nh_ptr->ok()) {
	  this->ros_cb_queue.callAvailable(ros::WallDuration(timeout));
  }

}

/////////////////////////////////////////////////////////////////////

void ActorPlugin::PublishActorInfo(ignition::math::Vector3d af_, ignition::math::Vector3d pos_,
    ignition::math::Vector3d sf_, double yaw_)
{

	ignition::math::Angle force_direction_ = atan2(af_.Y(), af_.X()) - yaw_;
	force_direction_.Normalize();

	ignition::math::Angle pos_direction_ = atan2(pos_.Y(), pos_.X()) - yaw_;
	pos_direction_.Normalize();

	ignition::math::Angle sf_direction_ = atan2(sf_.Y(), sf_.X()) - yaw_;
	sf_direction_.Normalize();

	geometry_msgs::Twist actor_vel_twist;
	actor_vel_twist.linear.x = pos_.Length() * cos(pos_direction_.Radian());
	actor_vel_twist.linear.y = pos_.Length() * sin(pos_direction_.Radian());
	actor_vel_twist.linear.z = sf_.Length() * cos(sf_direction_.Radian());
	actor_vel_twist.angular.x = af_.Length() * cos(force_direction_.Radian());
	actor_vel_twist.angular.y = af_.Length() * sin(force_direction_.Radian());
	//if (this->actor->GetName()=="actor1"){
	//  ROS_ERROR("%s, force x: %lf, force y: %lf", this->actor->GetName().c_str(), actor_vel_twist.angular.x, actor_vel_twist.angular.y);
	//}
	actor_vel_twist.angular.z = sf_.Length()*sin(sf_direction_.Radian());
	vel_publisher.publish(actor_vel_twist);

}

/////////////////////////////////////////////////////////////////////
void ActorPlugin::getROSParameters(const ros::NodeHandlePtr ros_nh_const_ptr){

	assert(ros_nh_const_ptr->getParam("/SOCIAL_FORCE_FACTOR", 	social_force_factor));
	assert(ros_nh_const_ptr->getParam("/DESIRED_FORCE_FACTOR", 	desired_force_factor));
	assert(ros_nh_const_ptr->getParam("/OBSTACLE_FORCE_FACTOR", 	obstacle_force_factor));
	assert(ros_nh_const_ptr->getParam("/MAX_SPEED", 				max_speed));
	assert(ros_nh_const_ptr->getParam("/MAX_ANGLE_UPDATE", 		max_angle_update));
	assert(ros_nh_const_ptr->getParam("/DODGING_RIGHT", 			dodging_right));
	assert(ros_nh_const_ptr->getParam("/TB3_AS_ACTOR", 			tb3_as_actor));
	assert(ros_nh_const_ptr->getParam("/TB3_NAME", 				tb3_name));
	assert(ros_nh_const_ptr->getParam("/ANIMATION_FACTOR", 		animation_factor));
	assert(ros_nh_const_ptr->getParam("/SF_LAMBDA_IMPORTANCE",	sf_lambda_importance));
	assert(ros_nh_const_ptr->getParam("/SF_GAMMA", 				sf_gamma));
	assert(ros_nh_const_ptr->getParam("/SF_N", 					sf_n));
	assert(ros_nh_const_ptr->getParam("/SF_N_PRIME", 				sf_n_prime));
	assert(ros_nh_const_ptr->getParam("/SF_DISTANCE_TH", 			sf_distance_threshold));
	assert(ros_nh_const_ptr->getParam("/NEIGHBOR_RANGE", 			neighbor_range));
	assert(ros_nh_const_ptr->getParam("/DEPTH_FOV", 				depth_fov));
	assert(ros_nh_const_ptr->getParam("/FIXED_ACTOR_HEIGHT", 		fixed_actor_height));
	assert(ros_nh_const_ptr->getParam("/VEL_PARAM", 				vel_param));
	depth_fov = depth_fov/180.0*M_PI;

}


/////////////////////////////////////////////////
// TODO  A vary naive strategy need to be update
//void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
//{
//for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
//{
//physics::ModelPtr model = this->world->ModelByIndex(i);
//if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
//model->GetName()) == this->ignoreModels.end())
//{
//ignition::math::Vector3d offset = model->WorldPose().Pos() -
//this->actor->WorldPose().Pos();
//double modelDist = offset.Length();
//if (modelDist < 2.0)
//{
//double invModelDist = this->obstacleWeight / modelDist;
//offset.Normalize();
//offset *= invModelDist;
//_pos -= offset;
//}
//}
//}
//}

////useless now
//ignition::math::Vector3d ActorPlugin::ObstacleForce(ignition::math::Pose3d &_pose) const
//{
//ignition::math::Vector3d minDiff;
//double minDistanceSquared = INFINITY;

//for(unsigned int i = 0; i < this->world->ModelCount(); i++) {
//physics::ModelPtr currentObstacle = this->world->ModelByIndex(i);

//if (currentObstacle->HasType(physics::Base::EntityType::ACTOR)) {
//continue;
//}

//double distance = currentObstacle->WorldPose().Pos().Distance(_pose.Pos());
//double distanceSquared = distance * distance;
//if (distanceSquared < minDistanceSquared) {
//minDistanceSquared = distanceSquared;
//minDiff = distance;
//}
//}
//double minDistance = sqrt(minDistanceSquared);
//double forceAmount = exp(-minDistance / 0.8);
//return forceAmount * minDiff.Normalize();
//}

///////////////////////////////////////////////////
//// deprecated
//void ActorPlugin::ChooseNewTarget()
//{
//ignition::math::Vector3d newTarget(this->target);
//while ((newTarget - this->target).Length() < 2.0)
//{
//newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
//newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

//for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
//{
//double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
//- newTarget).Length();
//if (dist < 2.0)
//{
//newTarget = this->target;
//break;
//}
//}
//}
//this->target = newTarget;
//}
