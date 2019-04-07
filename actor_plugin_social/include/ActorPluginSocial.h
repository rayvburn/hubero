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

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include "Enums.h"
#include "CommonInfo.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include "sfm/core/SocialForceModel.h"
#include "sfm/core/SFMDebug.h"

// -------------------------

#define VISUALIZE_SFM
//#define CREATE_ROS_NODE
#define CREATE_ROS_INTERFACE

// -------------------------

#ifdef VISUALIZE_SFM
	//#define VIS_SFM_GRID
	#define VIS_SFM_POINT
#endif

// -------------------------

#ifdef CREATE_ROS_NODE
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#endif

#ifdef CREATE_ROS_INTERFACE
#include "ActorROSInterface.h"
#endif

// -------------------------

#ifdef VISUALIZE_SFM

	#ifdef VIS_SFM_POINT
	#include <SFMVisPoint.h>
	#elif defined(VIS_SFM_GRID)
	#include <SFMVisGrid.h>
	#endif

#endif

// -------------------------

//#define INFLATE_BOUNDING_BOX
//#define INFLATE_BOUNDING_CIRCLE
#define INFLATE_BOUNDING_ELLIPSE

// -------------------------

#if		defined(INFLATE_BOUNDING_BOX)
#include "inflation/Box.h"
#elif 	defined(INFLATE_BOUNDING_CIRCLE)
#include "BoundingCircle.h"
#include "inflation/Circle.h"
#elif 	defined(INFLATE_BOUNDING_ELLIPSE)
#include "BoundingEllipse.h"
#include "inflation/Ellipse.h"
#endif

// -------------------------

//static std::vector<ignition::math::Vector3d> lin_vels_vector;
//static std::map<std::string, unsigned int> map_of_names;

#define REFACTOR_COMMON

namespace gazebo
{


// ACTOR MODEL TYPE is:  -------- MODEL TYPE -------- 32771



// -------------------------

  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Unherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    private: void chooseNewTarget(const common::UpdateInfo &_info);

    /// \brief Time of the last new target selection
	private: common::Time last_target_selection_time;

    /// \brief Helper function to check if target is still
    /// reachable (for example after addition of a new model
    /// a current target may not be reachable any more)
    private: bool isTargetStillReachable(const common::UpdateInfo &_info);

    /// \brief Time of the last reachability test.
	private: common::Time last_reachability_time;

    /// \brief Helper function to check if target is not
    /// reached for a certain amount of time
    private: bool isTargetNotReachedForTooLong(const common::UpdateInfo &_info) const;

    private: bool doesBoundingBoxContainPoint(const ignition::math::Box &_bb, const ignition::math::Vector3d &_pt) const;

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Desired velocity of the actor
    private: ignition::math::Vector3d velocity_desired;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animation_factor = 1.0;

    /// \brief Time of the last update.
    private: common::Time last_update;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;

    // ----------------------------------------------------------

    /// \brief TODO
    private: bool ReadSDF();


    private: ActorUtils::CommonInfo actor_common_info;

#if	defined(INFLATE_BOUNDING_BOX)

    private: actor::inflation::Box bounding_box;
    private: static ignition::math::Box GenerateBoundingBox(const ignition::math::Pose3d &_actor_pose);

#elif defined(INFLATE_BOUNDING_CIRCLE)

    //private: ActorUtils::BoundingCircle bounding_circle;
    private: actor::inflation::Circle bounding_circle;

#elif defined(INFLATE_BOUNDING_ELLIPSE)

    //private: ActorUtils::BoundingEllipse bounding_ellipse;
    private: actor::inflation::Ellipse bounding_ellipse;

#endif


    /// \brief Pose of the actor
    private: ignition::math::Pose3d pose_actor;	// raw pose value has the offset
    private: inline void SetActorPose(const ignition::math::Pose3d &_pose);

    /// \brief Helper function that considers the offset of the actor's yaw and a roll
    /// 	   offset depending on current stance
    private: ignition::math::Vector3d UpdateActorOrientation();
    private: ignition::math::Vector3d UpdateActorOrientation(const ignition::math::Pose3d &_pose);

    /// \brief Type of current stance of the actor
    private: ActorStance stance_actor;

    /// \brief Current state of the actor
    private: ActorState state_actor;
    private: ActorState prev_state_actor;

    private: bool AlignToTargetDirection(ignition::math::Vector3d *_rpy);

    // handlers for each state
    private: void ActorStateAlignTargetHandler	(const common::UpdateInfo &_info);
    private: void ActorStateMoveAroundHandler	(const common::UpdateInfo &_info);
    private: void ActorStateFollowObjectHandler	(const common::UpdateInfo &_info);
    private: void ActorStateTeleoperationHandler(const common::UpdateInfo &_info);

    // functions invoked at start and in the end of each OnUpdate event
    private: double PrepareForUpdate(const common::UpdateInfo &_info);
    private: void ApplyUpdate(const common::UpdateInfo &_info, const double &_dist_traveled);

    /// \brief Last actor's pose
    private: ignition::math::Pose3d last_pose_actor;

    /// \brief Actual velocity of the actor
    private: ignition::math::Vector3d velocity_actual;

    private: std::array<ignition::math::Vector3d, 50> velocities_to_avg;

    /// \brief Helper function to calculate the velocity (if it is allowable)
    /// 	   allowable in terms of immediate jumps which are not permitted
    private: bool CalculateVelocity(const ignition::math::Vector3d &_pos, const double &_dt);

    /// \brief Social Force Model interface object
    private: sfm::core::SocialForceModel sfm;



#ifdef VISUALIZE_SFM

    private: void VisualizeForceField();

#ifdef VIS_SFM_POINT
    private: static SocialForceModel::SFMVisPoint sfm_vis;
    private: SocialForceModel::SFMVisPoint sf_vis;
#elif defined(VIS_SFM_GRID)
    private: static SocialForceModel::SFMVisGrid sfm_vis;
#endif

    private: static SocialForceModel::SFMVisGrid grid_vis;

#endif



#ifdef CREATE_ROS_NODE
    void PublishActorTf();
    std::unique_ptr<ros::NodeHandle> ros_nh;
    ros::Publisher vis_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;
#endif

#ifdef CREATE_ROS_INTERFACE
    ActorUtils::ActorROSInterface ros_interface;
#endif

  };
}
#endif
