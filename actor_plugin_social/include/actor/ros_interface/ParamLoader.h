/*
 * ParamLoader.h
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ROS_INTERFACE_PARAMLOADER_H_
#define INCLUDE_ACTOR_ROS_INTERFACE_PARAMLOADER_H_

// C++ STL libraries
#include <vector>
#include <memory>
#include <tuple>
#include <string>
#include <limits>

// ROS headers
#include <ros/ros.h>
#include <XmlRpc.h>


namespace actor {
namespace ros_interface {

/* Helper class created to prevent the main one's (actor::core::Actor)
 * pollution with a plenty of parameter variables used only once
 * during initialization */
class ParamLoader {

public:

	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		std::vector<double> init_pose; 						// if empty - there will be .world file values chosen
		std::vector<double> init_target;					// if empty - there will be random values chosen
		unsigned short int	init_stance						= 0;

		std::string 		global_frame_name				= "world";
		double 				animation_factor 				= 4.5;				/// \brief Time scaling factor. Used to coordinate translational motion with the actor_ptr_'s walking animation.
		double 				animation_speed_rotation 		= 0.007;
		double 				target_tolerance 				= 1.25;
		double 				target_reach_max_time 			= 60.0;
		double 				target_reachable_check_period 	= 2.0;
		bool 				limit_actors_workspace			= true;
	    std::vector<double> world_bound_x					{-3.20, +3.80};
	    std::vector<double> world_bound_y					{-10.20, +3.80};

	} ActorParams;

	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		unsigned short int 	bounding_type 					= 2;
		double				circle_radius					= 0.5;
		std::vector<double> box_size						{0.45, 0.45, 1.00};
		std::vector<double> ellipse							{1.00, 0.80, 0.35, 0.00};

	} InflatorParams;

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		double 				fov 						= 2.00;
		double 				max_speed 					= 1.50;
		double 				mass 						= 80.0;
		double 				maneuverability				= 6.5;
		double 				internal_force_factor 		= 100.0;
		double 				interaction_force_factor 	= 3000.0;
		double 				min_force 					= 300.0;
		double 				max_force 					= 2000.0;
		bool 				heterogenous_population 	= false;
		unsigned short int 	static_obj_interaction 		= 1;
		unsigned short int 	box_inflation_type 			= 0;
		unsigned short int	opposite_force				= 0;
		bool 				disable_interaction_forces	= false;

	} SfmParams;

	/// \brief Declaration of an SfmVisParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		double				markers_pub_period			= 0.15;
		bool				publish_grid				= true;
		double				grid_resolution				= 0.75;
		double 				grid_pub_period				= 0.25;

	} SfmVisParams;

	/// \brief Struct containing name of the world model (in a `combined` form)
	/// along with a width of a single wall
	typedef struct {
		std::string 		name;
		double 				wall_width;
	} WorldModel;

	/// \brief Declaration of an SfmDictionary typedef'ed struct;
	/// default values are provided
	typedef struct {
		WorldModel world_model;
		std::vector<std::string> ignored_models_;
		std::vector<std::tuple<std::string, int, double> > model_description;
	} SfmDictionary;

	/// \brief Declaration of a BehaviourParams typedef'ed struct;
	/// default values are provided
	typedef struct {
		double			force_factor					= 1.0;
		double			turn_left 						= 500.0;
		double			turn_left_accelerate_turn 		= 500.0;
		double 			turn_left_accelerate_acc 		= 625.0;
		double 			accelerate 						= 500.0;
		double 			turn_right_accelerate_turn 		= 500.0;
		double			turn_right_accelerate_acc 		= 625.0;
		double			turn_right 						= 800.0;
		double 			turn_right_decelerate_turn 		= 500.0;
		double 			turn_right_decelerate_dec 		= 625.0;
		double 			stop 							= 500.0;
		double			decelerate 						= 500.0;
	} BehaviourParams;

	// -------------------------

	/// \brief Default constructor
	ParamLoader();

	/// \brief Sets main namespace of parameters to search within,
	/// it is meant to be an actor's name (if each actor has
	/// different parameters);
	/// for further explanation see `namespace and prefixes
	/// explanation` section in this file
	void setNamespace(const std::string &ns);

	/// \brief Sets namespace of actor's parameters to search within,
	/// for further explanation see `namespace and prefixes
	/// explanation` section in this file
	void setActorParamsPrefix(const std::string &actor_prefix);

	/// \brief Sets namespace of SFM's parameters to search within,
	/// for further explanation see `namespace and prefixes
	/// explanation` section in this file
	void setSfmParamsPrefix(const std::string &sfm_prefix);

	/// \brief Sets namespace of the Behaviour-related parameters to search within,
	/// for further explanation see `namespace and prefixes
	/// explanation` section in this file
	void setBehaviourParamsPrefix(const std::string &beh_prefix);

	/// \brief Loads parameters into typedef'ed struct
	/// instances (marked as private in this class)
	void loadParameters(const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Returns actor's parameters
	/// \return ActorParams struct with non-default values
	/// if parameters were successfully decoded
	ActorParams getActorParams() const;

	/// \brief Returns actor bounding's parameters
	/// \return InflatorParams struct with non-default values
	/// if parameters were successfully decoded
	InflatorParams getActorInflatorParams() const;

	/// \brief Returns SFM's parameters
	/// \return SfmParams struct with non-default values
	/// if parameters were successfully decoded
	SfmParams getSfmParams() const;

	/// \brief Returns SFM's visualization parameters
	/// \return SfmParams struct with non-default values
	/// if parameters were successfully decoded
	SfmVisParams getSfmVisParams() const;

	/// \brief Returns SFM dictionary mutable reference
	/// \return Non-empty SfmDictionary struct
	/// if parameters were successfully decoded
	SfmDictionary& getSfmDictionary() const;

	/// \brief Returns Behaviours parameters
	/// \return BehaviourParams struct with non-default values
	/// if parameters were succesfully decoded
	BehaviourParams getBehaviourParams() const;

	/// \brief Default destructor
	virtual ~ParamLoader();

private:

	/// \brief Helper function which loads actor's parameters
	/// into typedef'ed struct
	void loadActorParams (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which loads actor bounding's
	/// parameters into typedef'ed struct
	void loadActorInflatorParams (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which loads SFM's parameters
	/// into typedef'ed struct
	void loadSfmParams (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which loads SFM's visualization
	/// parameters into typedef'ed struct
	void loadSfmVisParams (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which loads SFM's dictionary
	/// into typedef'ed struct
	void loadSfmDictionary (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which loads Behaviours' parameters
	/// into typedef'ed struct
	void loadBehaviourParams (const std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Helper function which converts a given string
	/// into namespace pattern - '/' at the end
	/// \return New string modified appropriately
	inline std::string convertToNamespaceConvention(const std::string &str);

	/// \brief Helper function which checks if given `str`
	/// fits namespace pattern, (if needed) modifies
	/// a copy of it and assigns new string to a `ns`
	/// \brief [param in] string instance to be updated (set)
	/// \brief [param in] new name for a namespace
	inline void setLocalNs(std::string &ns, const std::string &str);

	/// \brief Helper function which converts model description
	/// loaded from YAML given in XmlRpcValue's format
	/// to std::tuple;
	/// NOTE: XmlRpc::XmlRpcValue passed to function must
	/// be non-const
	std::tuple<std::string, int, double> convertModelDescriptionToTuple(XmlRpc::XmlRpcValue &sublist);

	/// \brief Helper function which converts world model data
	/// loaded from YAML.
	/// \details See \ref convertModelDescriptionToTuple
	WorldModel convertWorldModelToStruct(XmlRpc::XmlRpcValue &sublist);

	/// \brief Helper function which sets smaller axis-bound
	/// value to 0 element and bigger to 1st;
	/// such an order is imposed in actor::core::Actor
	void sortVectorValues(std::vector<double> &vector);

	/// \brief Main namespace name (for example actor's name)
	std::string ns_;

	/// \brief Actor's parameters namespace prefix
	std::string actor_ns_prefix_;

	/// \brief SFM's parameters namespace prefix
	std::string sfm_ns_prefix_;

	/// \brief Behaviour-related parameters namespace prefix
	std::string beh_ns_prefix_;

	/// \brief Struct storing actor's parameters
	ActorParams params_actor_;

	/// \brief Struct storing actor bounding's parameters
	InflatorParams params_actor_bounding_;

	/// \brief Struct storing SFM's parameters
	SfmParams params_sfm_;

	/// \brief Struct storing behaviours' parameters
	BehaviourParams params_beh_;

	/// \brief Struct storing SFM's dictionary;
	/// marked static as dictionary is shared
	/// among all actors
	static SfmDictionary dict_sfm_;

	/// \brief Struct storing SFM's dictionary;
	/// marked static as dictionary is shared
	/// among all actors
	static SfmVisParams params_sfm_vis_;

	/// \brief Helper flag to prevent loading
	/// dictionary and vis parameters multiple times;
	/// marked static just like SFM's dictionary
	static bool dict_vis_loaded_;

	/*
	 * NOTE: setting namespace and prefixes is NOT necessary
	 * Namespace and prefixes explanation:
	 *
	 * default (blank actor's namespace and actor's prefix
	 * /gazebo/[node name]/[param name], for example:
	 * /gazebo/actor_plugin_ros_interface/general/animation_factor: 4.5
	 *
	 * extended version:
	 * /gazebo/actor_plugin_ros_interface/[ns=actor_id]/[actor_ns_prefix]/general/animation_factor: 4.5
	 * /gazebo/actor_plugin_ros_interface/[ns=actor_id]/[sfm_ns_prefix]/algorithm/fov: 2.0
	 *
	 * NOTE: if a prefix or a namespace is set, it must end with a '/' character -
	 * this is managed by `convertToNamespaceConvention()` function
	 */

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_ROS_INTERFACE_PARAMLOADER_H_ */
