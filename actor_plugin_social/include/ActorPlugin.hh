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
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <SocialForceModel.h>

#include <actor_sim_srv/SetPose.h>
#include <actor_sim_srv/GetVel.h>

namespace gazebo
{
  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {

    public: 

	  /// \brief Constructor
      ActorPlugin();

      /// \brief Load the actor plugin.
      /// \param[in] _model Pointer to the parent model.
      /// \param[in] _sdf Pointer to the plugin's SDF elements.
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:

      /// \brief Function that is called every update cycle.
      /// \param[in] _info Timing information
      void OnUpdate(const common::UpdateInfo &_info);

      bool SetPoseCallback(actor_sim_srv::SetPose::Request&, actor_sim_srv::SetPose::Response&);

      bool SetTargetCallback(actor_sim_srv::SetPose::Request&, actor_sim_srv::SetPose::Response&);

      bool GetVelCallback(actor_sim_srv::GetVel::Request&, actor_sim_srv::GetVel::Response&);

      ignition::math::Vector3d CallActorVelClient(std::string) const;

      void PublishActorInfo(ignition::math::Vector3d, ignition::math::Vector3d, ignition::math::Vector3d, double);

      /// \brief Helper function to choose a new target location
      void ChooseNewTarget();

      /// \brief Helper function to avoid obstacles. This implements a very
      /// simple vector-field algorithm.
      /// \param[in] _pos Direction vector that should be adjusted according
      /// to nearby obstacles.
      void HandleObstacles(ignition::math::Vector3d &_pos);

      //double socialForceFactor;
      //double desiredForceFactor;
      //double obstacleForceFactor;

      /// Compute the social force.
      ignition::math::Vector3d SocialForce(ignition::math::Pose3d &_pose, ignition::math::Vector3d _velocity) const;

      /// Compute the obstacle force.
      ignition::math::Vector3d ObstacleForce(ignition::math::Pose3d &_pose) const;

      void QueueThread();

      // param list
      void getROSParameters(const ros::NodeHandlePtr);

      /// \brief A node use for ROS transport
      // std::shared_ptr<ros::NodeHandle> rosNode;
      ros::NodeHandlePtr ros_nh_ptr;

      ros::ServiceServer set_pose_srv;

      ros::ServiceServer set_target_srv;

      ros::ServiceServer get_vel_srv;

      // ros::ServiceClient GetVelClient;

      /// \brief Pointer to the parent actor.
      physics::ActorPtr actor;

      /// \brief Pointer to the world, for convenience.
      physics::WorldPtr world;

      /// \brief Velocity of the actor
      ignition::math::Vector3d velocity;

      /// \brief Angular Velocity of the yaw axis
      double yaw_vel;

      /// \brief Max velocity of the actor
      double v_max;

      /// \brief The direction the actor will dodge in, will dodge right if true or by default
      //bool dodgingRight;

      /// \brief List of connections
      std::vector<event::ConnectionPtr> connections;

      /// \brief Current target location
      ignition::math::Vector3d target;

      /// \brief Start location
      ignition::math::Vector3d start_location;

      /// \brief Target location weight (used for vector field)
      //double targetWeight = 1.0;

      /// \brief Obstacle weight (used for vector field)
      //double obstacleWeight = 1.0;

      /// \brief Time scaling factor. Used to coordinate translational motion
      /// with the actor's walking animation.
      //double animationFactor = 1.0;

      /// \brief Time of the last update.
      common::Time last_update;

      /// \brief List of models to ignore. Used for vector field
      std::vector<std::string> ignored_models;

      /// \brief Custom trajectory info.
      physics::TrajectoryInfoPtr trajectory_info;

      ros::Publisher vel_publisher;

      /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue ros_cb_queue;

      /// \brief A thread the keeps running the rosQueue
      std::thread ros_queue_thread;

      double social_force_factor;
      double desired_force_factor;
      double obstacle_force_factor;
      double max_speed;
      double max_angle_update;
      bool dodging_right;
      bool tb3_as_actor;
      std::string tb3_name;
      double animation_factor;
      double sf_lambda_importance;
      double sf_gamma;
      double sf_n;
      double sf_n_prime;
      double sf_distance_threshold;
      double neighbor_range;
      double depth_fov;
      double fixed_actor_height; 
      double vel_param;

  };

}
#endif
