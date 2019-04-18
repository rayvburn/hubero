/*
 * ParamLoader.cpp
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#include "ros_interface/ParamLoader.h"
#include <iostream> // debugging

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

// static class variables
ParamLoader::SfmDictionary actor::ros_interface::ParamLoader::dict_sfm_;
ParamLoader::SfmVisParams  actor::ros_interface::ParamLoader::params_sfm_vis_;
bool actor::ros_interface::ParamLoader::dict_vis_loaded_ = false;

// ------------------------------------------------------------------- //

ParamLoader::ParamLoader() { }

// ------------------------------------------------------------------- //

void ParamLoader::setNamespace(const std::string &ns) {
	setLocalNs(ns_, ns);
}

// ------------------------------------------------------------------- //

void ParamLoader::setActorParamsPrefix(const std::string &actor_prefix) {
	setLocalNs(actor_ns_prefix_, actor_prefix);
}

// ------------------------------------------------------------------- //

void ParamLoader::setSfmParamsPrefix(const std::string &sfm_prefix) {
	setLocalNs(sfm_ns_prefix_, sfm_prefix);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadParameters(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	if ( nh_ptr == nullptr ) {
		std::cout << "\n\nParamLoader::loadParameters() - NodeHandle null!\n\n";
		return;
	}

	loadActorParams(nh_ptr);
	loadSfmParams(nh_ptr);

	// dictionary is shared between actors, let it load only once
	if ( !dict_vis_loaded_ ) {
		loadSfmVisParams(nh_ptr);
		loadSfmDictionary(nh_ptr);
		dict_vis_loaded_ = true;
	}

}

// ------------------------------------------------------------------- //

ParamLoader::ActorParams ParamLoader::getActorParams() const {
	return (params_actor_);
}

// ------------------------------------------------------------------- //

ParamLoader::SfmParams ParamLoader::getSfmParams() const {
	return (params_sfm_);
}

// ------------------------------------------------------------------- //

ParamLoader::SfmVisParams ParamLoader::getSfmVisParams() const {
	return (params_sfm_vis_);
}

// ------------------------------------------------------------------- //

ParamLoader::SfmDictionary ParamLoader::getSfmDictionary() const {
	return (dict_sfm_);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadActorParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// initialization section
	int temp_int; 	// variable to store an int further converted to unsigned short int

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/pose", params_actor_.init_pose) ) { }

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/stance", temp_int) ) {
		params_actor_.init_stance = static_cast<unsigned short int>(temp_int);
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// general section
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/bounding_type", temp_int) ) {
		params_actor_.bounding_type = static_cast<unsigned short int>(temp_int);
	}

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_factor", params_actor_.animation_factor) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_speed_rotation", params_actor_.animation_speed_rotation) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_tolerance", params_actor_.target_tolerance) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reach_max_time", params_actor_.target_reach_max_time) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reachable_check_period", params_actor_.target_reachable_check_period) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_x", params_actor_.world_bound_x) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_y", params_actor_.world_bound_y) ) { }

	sortVectorValues(params_actor_.world_bound_x);
	sortVectorValues(params_actor_.world_bound_y);

//	std::cout << "WORLD BOUND_X - size: " << params_actor_.world_bound_x.size() << "\n";
//	for ( size_t i = 0; i < params_actor_.world_bound_x.size(); i++ ) {
//		std::cout << i << "\t" << params_actor_.world_bound_x.at(i) << std::endl;
//	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM params section
	int temp_int; 	// variable to store an int further converted to unsigned short int

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/fov", params_sfm_.fov) ) {	}
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_speed", params_sfm_.max_speed) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/mass", params_sfm_.mass) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/internal_force_factor", params_sfm_.internal_force_factor) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/interaction_force_factor", params_sfm_.interaction_force_factor) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/min_force", params_sfm_.min_force) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_force", params_sfm_.max_force) ) { }

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/static_obj_interaction", temp_int) ) {
		params_sfm_.static_obj_interaction = static_cast<unsigned short int>(temp_int);
	}

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/heterogenous_population", params_sfm_.heterogenous_population) ) { }

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/box_inflation_type", temp_int) ) {
		params_sfm_.box_inflation_type = static_cast<unsigned short int>(temp_int);
	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmVisParams (const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM Visualization params section
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/markers_pub_period", params_sfm_vis_.markers_pub_period) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/publish_grid", params_sfm_vis_.publish_grid) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/grid_resolution", params_sfm_vis_.grid_resolution) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/grid_pub_period", params_sfm_vis_.grid_pub_period) ) { }
}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmDictionary(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM dictionary section
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "world_dictionary/ignored_models", dict_sfm_.ignored_models_) ) { }

	XmlRpc::XmlRpcValue list;
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "world_dictionary/model_description", list) ) {

		// temporary local variables to decode YAML list and convert it to a tuple
		XmlRpc::XmlRpcValue sublist;

		for (int i = 0; i < list.size(); i++) {
		      dict_sfm_.model_description.push_back( convertModelDescriptionToTuple(list[i]) );
//			  std::cout << "\t" << i << "\t" << "name: " << std::get<0>(dict_sfm_.model_description.at(i)) << "\tmass: " << std::get<1>(dict_sfm_.model_description.at(i)) << "\timmunity: " << std::get<2>(dict_sfm_.model_description.at(i)) << std::endl;
		}

	}

}

// ------------------------------------------------------------------- //

inline std::string ParamLoader::convertToNamespaceConvention(const std::string &str) {

	std::string temp(str);
	temp.append("/");
	return (temp);

}

// ------------------------------------------------------------------- //

inline void ParamLoader::setLocalNs(std::string &ns, const std::string &str) {

	// check if the last character is a '/'
	if ( str.back() != '/' ) {
		ns = convertToNamespaceConvention(str);
		return;
	}
	ns = str;

}

// ------------------------------------------------------------------- //

std::tuple<std::string, int, double> ParamLoader::convertModelDescriptionToTuple(XmlRpc::XmlRpcValue &sublist) {

	std::string name = static_cast<std::string>( sublist["name"] );
	int mass = static_cast<int>( sublist["mass"] );
	double immunity = static_cast<double>( sublist["immunity"] );

	return ( std::make_tuple(name, mass, immunity) );

}

// ------------------------------------------------------------------- //

void ParamLoader::sortVectorValues(std::vector<double> &vector) {

	if ( vector.at(0) > vector.at(1) ) {

		/* there is a need to store a smaller world_bound_AXIS value as 0-indexed element */
		double temp_x_min = vector.at(0);
		double temp_x_max = vector.at(1);

		vector.at(0) = temp_x_max;
		vector.at(1) = temp_x_min;

	} else if ( std::fabs( vector.at(0) - vector.at(1) ) < 1e-06 ) {

		// empty world, give an error message
		std::cout << "ERROR - wrong world bound values - world is empty!" << std::endl;

	}

}

// ------------------------------------------------------------------- //

ParamLoader::~ParamLoader() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
