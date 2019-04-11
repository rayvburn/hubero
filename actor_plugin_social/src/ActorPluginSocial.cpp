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

//  	ignition::math::Vector3d init_orient = this->actor->WorldPose().Rot().Euler();
//  	ignition::math::Pose3d init_pose;
//  	ignition::math::Vector3d init_position;
//  	init_position = this->actor->WorldPose().Pos();
//  	init_position.Z(1.2138);
//
//  	init_pose.Set(init_position,
//				  ignition::math::Quaterniond(init_orient.X() + IGN_PI/2,
//											  init_orient.Y(),
//											  // init_orient.Z() + 1.5707));
//											  init_orient.Z()));
//
//	// WARNING: initial pose changed!
//  	this->actor->SetWorldPose(init_pose, false, false);
//	std::cout << " -------- SET WorldPose() actor! -------- " << init_pose << std::endl;
//
//	// conversions between Euler and Quaternion will finally produce the result that converges to 0...
//	// above is deprecated, yaw will be set in each OnUpdate()
//
//	// Set last_pos_actor to prevent velocity shootout
//	last_pose_actor.Pos() = this->actor->WorldPose().Pos();
//	std::cout << " -------- SET last_pos_actor! -------- " << last_pose_actor.Pos() << std::endl;
//
//	actor_common_info.addActor(this->actor->GetName());
//
//#if	defined(INFLATE_BOUNDING_BOX)
//	bounding_box.updatePose(this->pose_actor);
//	actor_common_info.SetBoundingBox(bounding_box);
//#elif defined(INFLATE_BOUNDING_CIRCLE)
//	bounding_circle.setCenter(this->pose_actor.Pos());
//	bounding_circle.setRadius(0.75f);
//	actor_common_info.SetBoundingCircle(bounding_circle);
//#elif defined(INFLATE_BOUNDING_ELLIPSE)
//	bounding_ellipse.setPosition(this->pose_actor.Pos());
//	bounding_ellipse.setYaw(this->pose_actor.Rot().Yaw());
//	bounding_ellipse.setSemiMajorAxis(1.00);
//	bounding_ellipse.setSemiMinorAxis(0.80);
//	bounding_ellipse.setCenterOffset(ignition::math::Vector3d(0.35, 0.0, 0.0));
//	actor_common_info.setBoundingEllipse(bounding_ellipse);
//#endif
//
//
//	std::cout << " -------- ACTOR ID -------- " << actor_common_info.getActorID() << std::endl;
//	std::cout << " -------- MODEL TYPE -------- " << this->model->GetType() << std::endl;

	// WARNING: HARD-CODED target coord
	if ( this->actor->GetName() == "actor1" ) {
		this->target.X(+0.00);
		this->target.Y(-4.00);
	}

	//sfm.Init(80.0, 2.0, 1.0, this->world);

	actor_object.initGazeboInterface(actor, world);
	actor_object.setNewTarget(ignition::math::Pose3d(target, ignition::math::Quaterniond(0.0, 0.0, 0.0)));

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

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
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

	common::UpdateInfo info_cpy = _info;
	//std::cout << "BEF UPDATE" << std::endl;
	actor_object.executeTransitionFunction(_info);
	//std::cout << "UPDATED" << std::endl;


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
//
//#ifdef SILENT_
//	print_info = false;
//#endif


  	return; // OnUpdate testing

} /* OnUpdate */

// ===============================================================================================
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
