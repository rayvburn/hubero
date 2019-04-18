/*
 * ParamLoader.h
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_PARAMLOADER_H_
#define INCLUDE_ROS_INTERFACE_PARAMLOADER_H_

// C++ STL libraries
#include <vector>
#include <memory>
#include <tuple>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <XmlRpc.h>


namespace actor {
namespace ros_interface {

/* Helper class created to prevent the main one (actor::core::Actor)
 * pollution with a plenty of parameter variables used only once
 * during initialization */
class ParamLoader {

public:

	/// \brief Declaration of an ActorParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		unsigned short int 	bounding_type 					= 2;
		double 				animation_factor 				= 4.5;
		double 				animation_speed_rotation 		= 0.007;
		double 				target_tolerance 				= 1.25;
		double 				target_reach_max_time 			= 60.0;
		double 				target_reachable_check_period 	= 2.0;
	    std::vector<double> world_bound_x					{-3.20, +3.80};
	    std::vector<double> world_bound_y					{-10.20, +3.80};

	} ActorParams;

	/// \brief Declaration of an SfmParams typedef'ed struct;
	/// default values are provided
	typedef struct {

		double 				fov 						= 2.00;
		double 				max_speed 					= 1.50;
		double 				mass 						= 80.0;
		double 				internal_force_factor 		= 100.0;
		double 				interaction_force_factor 	= 3000.0;
		double 				min_force 					= 300.0;
		double 				max_force 					= 2000.0;
		bool 				heterogenous_population 	= false;
		unsigned short int 	static_obj_interaction 		= 1;
		unsigned short int 	box_inflation_type 			= 0;

	} SfmParams;

	/// \brief Declaration of an SfmDictionary typedef'ed struct;
	/// default values are provided
	typedef struct {
		std::vector<std::string> ignored_models_;
		std::vector<std::tuple<std::string, int, double> > model_description;
	} SfmDictionary;

	// -------------------------

	/// \brief Default constructor
	ParamLoader();

	void setNamespace(const std::string &ns);

	void setActorParamsPrefix(const std::string &actor_prefix);
	void setSfmParamsPrefix(const std::string &sfm_prefix);
	void setSfmDictionaryPrefix(const std::string &dictionary_prefix);

//	/// \brief Loads ParameterServer instances and invokes
//	/// appropriate Actor's setter methods
//	void loadParameters();
	void loadParameters(const std::shared_ptr<ros::NodeHandle> nh_ptr);

	ActorParams   getActorParams() 		const;
	SfmParams 	  getSfmParams() 		const;
	SfmDictionary getSfmDictionary() 	const;

	/// \brief Default destructor
	virtual ~ParamLoader();

private:

	void loadActorParams	(const std::shared_ptr<ros::NodeHandle> nh_ptr);
	void loadSfmParams		(const std::shared_ptr<ros::NodeHandle> nh_ptr);
	void loadSfmDictionary	(const std::shared_ptr<ros::NodeHandle> nh_ptr);

	inline std::string convertToNamespaceConvention(const std::string &str);
	inline void setLocalNs(std::string &ns, const std::string &str);

	/// \brief
	/// must be non-const
	std::tuple<std::string, int, double> convertToTuple(XmlRpc::XmlRpcValue &sublist);

	/*
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
	 * NOTE: if a prefix or a namespace is set, it must end with a '/' character
	 */

	std::string ns_;
	std::string actor_ns_prefix_;
	std::string sfm_ns_prefix_;
	std::string dict_ns_prefix_;
	ActorParams params_actor_;
	SfmParams params_sfm_;
	SfmDictionary dict_sfm_;

//	/// \brief Parameters from an `initialization` section
//	std::vector<double> init_pose_;
//	unsigned int init_stance_;
//
//	/// \brief Parameters from a `general` section
//	unsigned int bounding_type_;
//	double animation_factor_;
//	double animation_factor_rotation_;
//	double target_tolerance_;
//	double target_reach_max_time_;
//	double target_reachable_check_period_;
//	std::vector<double> world_bound_x_;
//	std::vector<double> world_bound_y_;

//public:
//
//	// -------------------------
//
//	struct {
//		int field = 0;
//	} ActorParamsStruct;
//
//	// -------------------------
//
//	struct {
//		double 				fov 						= 2.00;
//		double 				max_speed 					= 1.50;
//		double 				mass 						= 80.0;
//		double 				internal_force_factor 		= 100.0;
//		double 				interaction_force_factor 	= 3000.0;
//		double 				min_force 					= 300.0;
//		double 				max_force 					= 2000.0;
//		bool 				heterogenous_population 	= false;
//		unsigned short int 	static_obj_interaction 		= 1;
//		unsigned short int 	box_inflation_type 			= 0;
//	} SfmParamsStruct;
//
//	// -------------------------
//
//	struct {
//		int field = 1;
//	} SfmDictStruct;
//
//	// -------------------------

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_PARAMLOADER_H_ */
