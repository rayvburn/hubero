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

ParamLoader::ParamLoader() {
	// TODO Auto-generated constructor stub

}

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

void ParamLoader::setSfmDictionaryPrefix(const std::string &dictionary_prefix) {
	setLocalNs(dict_ns_prefix_, dictionary_prefix);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadParameters(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	if ( nh_ptr == nullptr ) {
		std::cout << "\n\nParamLoader::loadParameters() - NodeHandle null!\n\n";
		return;
	}

	loadActorParams(nh_ptr);
	loadSfmParams(nh_ptr);
	loadSfmDictionary(nh_ptr);

}

// ------------------------------------------------------------------- //

void ParamLoader::loadActorParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// initialization section

	int i = 0;
	std::cout << "ACTOR PARAMETERS LOADING" << std::endl;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/pose", params_actor_.init_pose) ) {
		std::cout << i << "\tactor/initialization/pose LOADED: 1st elem" << params_actor_.init_pose.at(0) << std::endl;
	}
	i++;
	int temp_int;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/stance", temp_int) ) {
		params_actor_.init_stance = static_cast<unsigned short int>(temp_int);
		std::cout << i << "\tactor/initialization/stance LOADED: " << params_actor_.init_stance << std::endl;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// general section
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/bounding_type", temp_int) ) {
		params_actor_.bounding_type = static_cast<unsigned short int>(temp_int);
		std::cout << i << "\tactor/general/bounding_type LOADED: " << params_actor_.bounding_type << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_factor", params_actor_.animation_factor) ) {
		std::cout << i << "\tactor/general/animation_factor LOADED: " << params_actor_.animation_factor << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_speed_rotation", params_actor_.animation_speed_rotation) ) {
		std::cout << i << "\tactor/general/animation_speed_rotation LOADED: " << params_actor_.animation_speed_rotation << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_tolerance", params_actor_.target_tolerance) ) {
		std::cout << i << "\tactor/general/target_tolerance LOADED: " << params_actor_.target_tolerance << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reach_max_time", params_actor_.target_reach_max_time) ) {
		std::cout << i << "\tactor/general/target_reach_max_time LOADED: " << params_actor_.target_reach_max_time << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reachable_check_period", params_actor_.target_reachable_check_period) ) {
		std::cout << i << "\tactor/general/target_reachable_check_period LOADED: " << params_actor_.target_reachable_check_period << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_x", params_actor_.world_bound_x) ) {
		std::cout << i << "\tactor/general/world_bound_x LOADED: 1st elem: \n";// << params_actor_.world_bound_x << std::endl;
	}
	i++;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_y", params_actor_.world_bound_y) ) {
		std::cout << i << "\tactor/general/world_bound_y LOADED: 1st elem: \n";// << params_actor_.world_bound_y.at(i) << std::endl;
	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	int i = 0;
	std::cout << "SFM PARAMETERS LOADING" << std::endl;
	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// sfm section

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/fov", params_sfm_.fov) ) {
		std::cout << i << "\tsfm/algorithm/fov LOADED: " << params_sfm_.fov << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_speed", params_sfm_.max_speed) ) {
		std::cout << i << "\tsfm/algorithm/max_speed LOADED: " << params_sfm_.max_speed << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/mass", params_sfm_.mass) ) {
		std::cout << i << "\tsfm/algorithm/mass LOADED: " << params_sfm_.mass << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/internal_force_factor", params_sfm_.internal_force_factor) ) {
		std::cout << i << "\tsfm/algorithm/internal_force_factor LOADED: " << params_sfm_.internal_force_factor << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/interaction_force_factor", params_sfm_.interaction_force_factor) ) {
		std::cout << i << "\tsfm/algorithm/interaction_force_factor LOADED: " << params_sfm_.interaction_force_factor << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/min_force", params_sfm_.min_force) ) {
		std::cout << i << "\tsfm/algorithm/min_force LOADED: " << params_sfm_.min_force << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_force", params_sfm_.max_force) ) {
		std::cout << i << "\tsfm/algorithm/max_force LOADED: " << params_sfm_.max_force << std::endl;
	}
	i++;

	int temp_int;
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/static_obj_interaction", temp_int) ) {
		params_sfm_.static_obj_interaction = static_cast<unsigned short int>(temp_int);
		std::cout << i << "\tsfm/algorithm/static_obj_interaction LOADED: " << params_sfm_.static_obj_interaction << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/heterogenous_population", params_sfm_.heterogenous_population) ) {
		std::cout << i << "\tsfm/algorithm/heterogenous_population LOADED: " << params_sfm_.heterogenous_population << std::endl;
	}
	i++;

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/box_inflation_type", temp_int) ) {
		params_sfm_.box_inflation_type = static_cast<unsigned short int>(temp_int);
		std::cout << i << "\tsfm/algorithm/box_inflation_type LOADED: " << params_sfm_.box_inflation_type << std::endl;
	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmDictionary(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	if ( nh_ptr->getParam(ns_ + dict_ns_prefix_ + "world_dictionary/ignored_models", dict_sfm_.ignored_models_) ) {
//		std::cout << "world_dictionary/ignored_models - FOUND!" << std::endl;
//		for ( size_t i = 0; i < dict_sfm_.ignored_models_.size(); i++ ) {
//			std::cout << "\t" << i << "\t" << dict_sfm_.ignored_models_.at(i) << std::endl;
//		}
	}

	XmlRpc::XmlRpcValue list;
	if ( nh_ptr->getParam(ns_ + dict_ns_prefix_ + "world_dictionary/model_description", list) ) {

		std::cout << "\nworld_dictionary/model_description - FOUND! list size: " << list.size() << std::endl;

		// temporary local variables to decode YAML list and convert it to a tuple
		XmlRpc::XmlRpcValue sublist;
		std::string name;
		int mass;
		double immunity;

		for (int i = 0; i < list.size(); i++) {
		      sublist = list[i];
		      name = static_cast<std::string>(sublist["name"]);
		      mass = static_cast<int>(sublist["mass"]);
		      immunity = static_cast<double>(sublist["immunity"]);
		      std::cout << "\t" << i << "\t" << "name: " << name << "\tmass: " << mass << "\timmunity: " << immunity << std::endl;
		      dict_sfm_.model_description.push_back( convertModelDescriptionToTuple(list[i]) );
			  std::cout << "\t" << i << "\t" << "name: " << std::get<0>(dict_sfm_.model_description.at(i)) << "\tmass: " << std::get<1>(dict_sfm_.model_description.at(i)) << "\timmunity: " << std::get<2>(dict_sfm_.model_description.at(i)) << std::endl;
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

ParamLoader::~ParamLoader() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
