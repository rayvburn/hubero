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

using namespace gazebo; // FIXME?

GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

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
ActorPlugin::ActorPlugin() { }

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	this->sdf = _sdf;
	this->model = _model;
	this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
	this->world = this->actor->GetWorld();
	this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
		  std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

//	this->Reset();
//	if ( this->ReadSDF() ) {}

	// - - - - - - - - - - - - - - - - - - - - - -  - - - -- - - -- - - -- -  -- -

#if defined(CREATE_ROS_INTERFACE)
	ros_interface.Init(this->actor->GetName());
#endif

#ifdef VISUALIZE_SFM
	sfm_vis.Init("sfm", "map");
	grid_vis.Init("sfm_grid", "map");
#endif

	// - - - - - - - - - - - - - - - - - - - - - -  - - - -- - - -- - - -- -  -- -

    std::cout << "LOADED POSE: " << this->actor->WorldPose() << std::endl;

#ifndef ACTOR_SHARED_PTR
	actor_object.initGazeboInterface(actor, world);
	actor_object.initInflator(1.00, 0.80, 0.35, 0.0);
	actor_object.initRosInterface();
#else
//	actor::core::Actor obj;
//	actor_ptr_ = obj.shared_from_this();
	actor_ptr_ = std::make_shared<actor::core::Actor>();
	actor_ptr_->initGazeboInterface(actor, world);
	actor_ptr_->initInflator(1.00, 0.80, 0.35, 0.0);
	actor_ptr_->initRosInterface();
#endif

	ignition::math::Vector3d target_init;
	target_init.X(ignition::math::Rand::DblUniform(-3, 3.5));
	target_init.Y(ignition::math::Rand::DblUniform(-10, 2));
	target_init.Z(1.21);
#ifndef ACTOR_SHARED_PTR
	actor_object.setNewTarget(ignition::math::Pose3d(target_init, ignition::math::Quaterniond(0.0, 0.0, 0.0)));
#else
	actor_ptr_->setNewTarget(ignition::math::Pose3d(target_init, ignition::math::Quaterniond(0.0, 0.0, 0.0)));
#endif

}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
//  this->velocity_desired = 0.8;
//  this->last_update = 0;
//
//  if (this->sdf && this->sdf->HasElement("target"))
//    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
//  else
//    this->target = ignition::math::Vector3d(0, -5, 1.2138);
//
//  auto skelAnims = this->actor->SkeletonAnimations();
//  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
//  {
//    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
//  }
//  else
//  {
//    // Create custom trajectory
//    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
//    this->trajectoryInfo->type = WALKING_ANIMATION;
//    this->trajectoryInfo->duration = 1.0;
//
//    this->actor->SetCustomTrajectory(this->trajectoryInfo);
//  }
}

/////////////////////////////////////////////////

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
	}

	if ( !to_start ) {
		return;
	};

	// print only when vis is updated
	//if ( actor->GetName() == "actor1" ) {
		SfmSetPrintData(false);
	//}

#ifndef ACTOR_SHARED_PTR
	actor_object.executeTransitionFunction(_info);
#else
	actor_ptr_->executeTransitionFunction(_info);
#endif

	// ==================================================================================================
	/* ROS Interface & Visualization related */
//#ifdef CREATE_ROS_NODE
//	PublishActorTf();
//#elif defined(CREATE_ROS_INTERFACE)
//	ros_interface.PublishActorTf(this->pose_actor);
//	ros_interface.PublishTargetTf(this->target);
//	#ifdef INFLATE_BOUNDING_CIRCLE
//	ros_interface.PublishMarker(bounding_circle.getMarkerConversion());
//	#elif defined(INFLATE_BOUNDING_BOX)
//	//ros_interface.PublishMarker(sfm_vis.GetBBMarkerConversion(actor_common_info.GetBoundingBox()));
//	ros_interface.PublishMarker(bounding_box.getMarkerConversion());
//#elif defined(INFLATE_BOUNDING_ELLIPSE)
//	ros_interface.PublishMarker(bounding_ellipse.getMarkerConversion());
//	#endif
//#endif
//
//#if defined(VISUALIZE_SFM) && defined(VIS_SFM_POINT)
//	static common::Time vis_time;
//	static int counter = 0;
//
//	if ( (_info.simTime - vis_time).Double() >= 0.25 ) {
//
//		//std::cout << "ACTOR FOR VIS: " << this->actor_id << "\tname: " << this->actor->GetName() << std::endl;
////		if ( actor->GetName() == "actor1" ) {
////			SfmSetPrintData(false); // print in each iteration
////		}
//		VisualizeForceField();
//		if ( actor->GetName() == "actor1" ) {
//			SfmSetPrintData(true);
//		}
//		counter++;
//		if ( counter == 2 ) {
//			vis_time = _info.simTime;
//			counter = 0;
//		}
//
//	}
//#elif defined(VISUALIZE_SFM) && defined(VIS_SFM_GRID)
//
//	static common::Time vis_time;
//	if ( (_info.simTime - vis_time).Double() > 0.05 ) {
//		VisualizeForceField();
//		vis_time = _info.simTime;
//	}
//
//#endif

	// ==================================================================================================

//
//#ifdef SILENT_
//	print_info = false;
//#endif

//
//	// debugging purposes
//
//  	static common::Time print_time;
//
//  	if ( (_info.simTime - print_time).Double() > 0.2 ) {
//
//#ifndef SILENT_
//		if ( this->actor->GetName() == "actor1" ) {
//
//			print_info = true;
//			Print_Set(true);
//			print_time = _info.simTime;
//			std::cout << "\n\n**************************************************** ACTOR1 ***********************************************" << std::endl;
//			std::cout << "**** INITIAL pose: " << this->actor->WorldPose() << "\t\t actor1 velocity: \t" << this->velocity_actual << "\t target: " << this->target << std::endl;
//			ignition::math::Angle yaw_from_vel_world( std::atan2(this->velocity_actual.Y(), this->velocity_actual.X()) + (IGN_PI/2) );
//			yaw_from_vel_world.Normalize();
//			std::cout << "**** YAW vs VEL comparison\t\tyaw_from_vel_act: " << std::atan2(this->velocity_actual.Y(), this->velocity_actual.X()) << "\tyaw_from_vel_vec: " << std::atan2( lin_vels_vector[this->actor_id].Y(), lin_vels_vector[this->actor_id].X() ) << "\tyaw_from_vel_WORLD: " << yaw_from_vel_world.Radian() << "\tyaw_from_pose: " << this->actor->WorldPose().Rot().Euler().Z() << std::endl;
//		}
//#endif
//
//	} else {
//		print_info = false;
//	}

  	return; // OnUpdate testing

} /* OnUpdate */

// ===============================================================================================
// ===============================================================================================
// ===============================================================================================
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
							actor_common_info.getActorID());


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
		ignition::math::Angle yaw_world( this->pose_actor.Rot().Yaw() - IGN_PI_2);
		yaw_world.Normalize();
		bounding_ellipse.updatePose(ignition::math::Pose3d(	this->pose_actor.Pos(),
															ignition::math::Quaterniond(this->pose_actor.Rot().Roll(),
																						this->pose_actor.Rot().Pitch(),
																						yaw_world.Radian()) ));
		actor_common_info.setBoundingEllipse(bounding_ellipse);
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

		// don't calculate when no subscribing node
		if ( ros_interface.getGridSubscribersNum() > 0 ) {

			// std::cout << "SOCIAL FORCE GRID" << std::endl;
			grid_vis.CreateGrid(-5.0, 5.5, -12.0, 4.0, 0.75);

			ignition::math::Pose3d pose;
			ignition::math::Vector3d sf;

			while ( !grid_vis.IsWholeGridChecked() ) {

				pose = ignition::math::Pose3d(grid_vis.GetNextGridElement(), this->pose_actor.Rot());

				// Remember to artificially place the actor (along with his bounding) in current grid cell!
		#if	defined(INFLATE_BOUNDING_BOX)
				bounding_box.updatePose(pose);
				actor_common_info.SetBoundingBox(bounding_box);
		#elif defined(INFLATE_BOUNDING_CIRCLE)
				this->bounding_circle.setCenter(pose.Pos());
				this->actor_common_info.SetBoundingCircle(this->bounding_circle);
		#elif defined(INFLATE_BOUNDING_ELLIPSE)
				ignition::math::Angle yaw_world( this->pose_actor.Rot().Yaw() - IGN_PI_2);
				yaw_world.Normalize();
				bounding_ellipse.updatePose(ignition::math::Pose3d(	pose.Pos(),
																	ignition::math::Quaterniond(this->pose_actor.Rot().Roll(),
																								this->pose_actor.Rot().Pitch(),
																								yaw_world.Radian()) ));

				this->actor_common_info.setBoundingEllipse(bounding_ellipse);
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


}

#endif

// ===============================================================================================
// ===============================================================================================
