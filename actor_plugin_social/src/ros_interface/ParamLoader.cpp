/*
 * ParamLoader.cpp
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#include "ros_interface/ParamLoader.h"

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
	loadActorParams(nh_ptr);
	loadSfmParams(nh_ptr);
	loadSfmDictionary(nh_ptr);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadActorParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	if ( nh_ptr == nullptr ) {
		std::cout << "\n\nParamLoader::loadActorParams() - NodeHandle null!\n\n";
		return;
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// initialization section

//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/pose", init_pose) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/stance", init_stance) ) {
//
//	}

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// general section


//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/bounding_type", general_bounding_type) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_factor", general_animation_factor) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_speed_rotation", general_animation_speed_rotation) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_tolerance", general_target_tolerance) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reach_max_time", general_reach_max_time) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reachable_check_period", general_target_reachable_check_period) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_x", general_world_bound_x) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/world_bound_y", general_world_bound_y) ) {
//
//	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// sfm section

//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/fov", params_sfm_.fov) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_speed", params_sfm_.max_speed) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/mass", params_sfm_.mass) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/internal_force_factor", params_sfm_.internal_force_factor) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/interaction_force_factor", params_sfm_.interaction_force_factor) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/min_force", params_sfm_.min_force) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_force", params_sfm_.max_force) ) {
//
//	}
//
//	int temp_int;
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/mass", temp_int) ) {
//		params_sfm_.static_obj_interaction = static_cast<unsigned short int>(temp_int);
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/heterogenous_population", params_sfm_.heterogenous_population) ) {
//
//	}
//
//	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/box_inflation_type", temp_int) ) {
//		params_sfm_.box_inflation_type = static_cast<unsigned short int>(temp_int);
//	}

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
		      dict_sfm_.model_description.push_back( convertToTuple(list[i]) );
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

std::tuple<std::string, int, double> ParamLoader::convertToTuple(XmlRpc::XmlRpcValue &sublist) {

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
