/*
 * ParamLoader.cpp
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#include <actor/ros_interface/ParamLoader.h>
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

void ParamLoader::setBehaviourParamsPrefix(const std::string &beh_prefix) {
	setLocalNs(beh_ns_prefix_, beh_prefix);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadParameters(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	if ( nh_ptr == nullptr ) {
		std::cout << "\n\nParamLoader::loadParameters() - NodeHandle null!\n\n";
		return;
	}

	loadActorParams(nh_ptr);
	loadSfmParams(nh_ptr);
	loadActorInflatorParams(nh_ptr);

	// dictionary is shared between actors, let it load only once
	if ( !dict_vis_loaded_ ) {
		loadSfmVisParams(nh_ptr);
		loadSfmDictionary(nh_ptr);
		dict_vis_loaded_ = true;
	}

	loadBehaviourParams(nh_ptr);

}

// ------------------------------------------------------------------- //

ParamLoader::ActorParams ParamLoader::getActorParams() const {
	return (params_actor_);
}

// ------------------------------------------------------------------- //

ParamLoader::InflatorParams ParamLoader::getActorInflatorParams() const {
	return (params_actor_bounding_);
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

ParamLoader::SfmDictionary& ParamLoader::getSfmDictionary() const {
	return (dict_sfm_);
}

// ------------------------------------------------------------------- //

ParamLoader::BehaviourParams ParamLoader::getBehaviourParams() const {
	return (params_beh_);
}

// ------------------------------------------------------------------- //

void ParamLoader::loadActorParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// initialization section
	int temp_int; 	// variable to store an int further converted to unsigned short int

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/pose", params_actor_.init_pose) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/target", params_actor_.init_target) ) { }

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "initialization/stance", temp_int) ) {
		params_actor_.init_stance = static_cast<unsigned short int>(temp_int);
	}

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// general section

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/global_frame_name", params_actor_.global_frame_name) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_factor", params_actor_.animation_factor) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/animation_speed_rotation", params_actor_.animation_speed_rotation) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_tolerance", params_actor_.target_tolerance) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reach_max_time", params_actor_.target_reach_max_time) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/target_reachable_check_period", params_actor_.target_reachable_check_period) ) { }
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "general/limit_actors_workspace", params_actor_.limit_actors_workspace) ) { }
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

void ParamLoader::loadActorInflatorParams (const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// inflation section
	int temp_int; 	// variable to store an int further converted to unsigned short int

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "inflation/bounding_type", temp_int) ) {
		params_actor_bounding_.bounding_type = static_cast<unsigned short int>(temp_int);
	}
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "inflation/circle_radius", params_actor_bounding_.circle_radius) ) { }

	XmlRpc::XmlRpcValue list;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "inflation/box_size", list) ) {

		XmlRpc::XmlRpcValue sublist = list[0]; // only 1 element expected
		params_actor_bounding_.box_size.clear();
		double x_half = static_cast<double>( sublist["x_half"] );
		double y_half = static_cast<double>( sublist["y_half"] );
		double z_half = static_cast<double>( sublist["z_half"] );
		params_actor_bounding_.box_size.push_back(x_half);
		params_actor_bounding_.box_size.push_back(y_half);
		params_actor_bounding_.box_size.push_back(z_half);
//		std::cout << "box loader: " << "\tx: " << x_half << "\ty: " << y_half << "\tz: " << z_half << std::endl;

	}

	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "inflation/ellipse", list) ) {

		XmlRpc::XmlRpcValue sublist = list[0]; // only 1 element expected
		params_actor_bounding_.ellipse.clear();
		double semi_major 	= static_cast<double>( sublist["semi_major"] );
		double semi_minor 	= static_cast<double>( sublist["semi_minor"] );
		double offset_x 	= static_cast<double>( sublist["offset_x"] );
		double offset_y 	= static_cast<double>( sublist["offset_y"] );
		params_actor_bounding_.ellipse.push_back(semi_major);
		params_actor_bounding_.ellipse.push_back(semi_minor);
		params_actor_bounding_.ellipse.push_back(offset_x);
		params_actor_bounding_.ellipse.push_back(offset_y);
//		std::cout << "ellipse loader: " << "\tmajor: " << semi_major << "\tminor: " << semi_minor << "\toffset_x: " << offset_x << "\toffset_y: " << offset_y << std::endl;

	}

	calculateCostmapInflationRadius(nh_ptr);

	/*
	// footprint test
	XmlRpc::XmlRpcValue foot;
	if ( nh_ptr->getParam(ns_ + actor_ns_prefix_ + "inflation/footprint", foot) ) {
		// sublist
		XmlRpc::XmlRpcValue sublist;
		// elements in the vector
		for ( size_t i = 0; i < foot.size(); i++ ) {
			sublist = foot[i];
			for ( size_t j = 0; j < sublist.size(); j++ ) {

			}
		}
//		std::cout << "\n\n\nfootprint size: " << foot.size() << "\n\n\n" << std::endl;
//		XmlRpc::XmlRpcValue sublist = foot[0];
//		std::cout << "\n\n\nsublist[0] size: " << sublist.size() << "\n\n\n" << std::endl;
//		sublist = foot[1];
//		std::cout << "\n\n\nsublist[1] size: " << sublist.size() << "\n\n\n" << std::endl;
//		sublist = foot[2];
//		std::cout << "\n\n\nsublist[2] size: " << sublist.size() << "\n\n\n" << std::endl;
//		sublist = foot[3];
//		std::cout << "\n\n\nsublist[3] size: " << sublist.size() << "\n\n\n" << std::endl;
	}
	*/

	calculateActorFootprint(nh_ptr);

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmParams(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM params section
	int temp_int; 	// variable to store an int further converted to unsigned short int

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/fov", params_sfm_.fov) ) {	}
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/max_speed", params_sfm_.max_speed) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/mass", params_sfm_.mass) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/maneuverability", params_sfm_.maneuverability) ) { }
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

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/opposite_force", temp_int) ) {
		params_sfm_.opposite_force = static_cast<unsigned short int>(temp_int);
	}

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "algorithm/disable_interaction_forces", params_sfm_.disable_interaction_forces) ) { }

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmVisParams (const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM Visualization params section
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/markers_pub_period", params_sfm_vis_.markers_pub_period) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/publish_grid", params_sfm_vis_.publish_grid) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/grid_resolution", params_sfm_vis_.grid_resolution) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/grid_pub_period", params_sfm_vis_.grid_pub_period) ) { }

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/publish_potential", params_sfm_vis_.publish_potential) ) { }
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "visualization/potential_resolution", params_sfm_vis_.potential_resolution) ) { }

}

// ------------------------------------------------------------------- //

void ParamLoader::loadSfmDictionary(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -

	XmlRpc::XmlRpcValue list;

	// SFM dictionary section
	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "world_dictionary/world_model", list) ) {
		// assuming that world bounds are composed from a single model,
		// the `list` length can be acquired via the `size()` method
		dict_sfm_.world_model = convertWorldModelToStruct(list[0]);
		if ( dict_sfm_.world_model.name == "none" ) {
			dict_sfm_.world_model.name = "";
			dict_sfm_.world_model.wall_width = 0.0;
		}
	}

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "world_dictionary/ignored_models", dict_sfm_.ignored_models_) ) { }

	if ( nh_ptr->getParam(ns_ + sfm_ns_prefix_ + "world_dictionary/model_description", list) ) {

		for (int i = 0; i < list.size(); i++) {
		      dict_sfm_.model_description.push_back( convertModelDescriptionToTuple(list[i]) );
//			  std::cout << "\t" << i << "\t" << "name: " << std::get<0>(dict_sfm_.model_description.at(i)) << "\tmass: " << std::get<1>(dict_sfm_.model_description.at(i)) << "\timmunity: " << std::get<2>(dict_sfm_.model_description.at(i)) << std::endl;
		}

	}

}

// ------------------------------------------------------------------- //

void ParamLoader::loadBehaviourParams (const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// - - - - - - - - - - - - - - - - - - - - - - - - -
	// SFM Visualization params section
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/force_factor", params_beh_.force_factor) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_left", params_beh_.turn_left) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_left_accelerate_turn", params_beh_.turn_left_accelerate_turn) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_left_accelerate_acc", params_beh_.turn_left_accelerate_acc) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/accelerate", params_beh_.accelerate) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_right_accelerate_turn", params_beh_.turn_right_accelerate_turn) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_right_accelerate_acc", params_beh_.turn_right_accelerate_acc) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_right", params_beh_.turn_right) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_right_decelerate_turn", params_beh_.turn_right_decelerate_turn) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/turn_right_decelerate_dec", params_beh_.turn_right_decelerate_dec) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/stop", params_beh_.stop) ) { }
	if ( nh_ptr->getParam(ns_ + beh_ns_prefix_ + "behaviour/decelerate", params_beh_.decelerate) ) { }

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

ParamLoader::WorldModel ParamLoader::convertWorldModelToStruct(XmlRpc::XmlRpcValue &sublist) {

	ParamLoader::WorldModel world_model;

	world_model.name = static_cast<std::string>( sublist["name"] );
	world_model.wall_width = static_cast<double>( sublist["wall_width"] );

	return (world_model);

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

void ParamLoader::calculateCostmapInflationRadius(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// stores tolerance parameter (default value is invalid)
	float inflation = -1.0f;

	// helper variables (to not allocate them inside switch statement)
	double size   = -1.0;
	double size_x = -1.0;
	double size_y = -1.0;

	/* See "gazebo_ros_people_sim/actor_plugin_social/include/core/Enums.h"
	 * for description. If some changes in "Enums.h" were applied the 'switch'
	 * statement's cases must be adjusted accordingly. */
	switch ( params_actor_bounding_.bounding_type ) {

	/* ACTOR_BOUNDING_BOX */
	case(actor::ActorBoundingType::ACTOR_BOUNDING_BOX):

		size_x = params_actor_bounding_.box_size.at(0);
		size_y = params_actor_bounding_.box_size.at(1);
		size = std::max(size_x, size_y);
		inflation = 2.15f * size * (std::sqrt(2));
		break;

	/* ACTOR_BOUNDING_CIRCLE */
	case(actor::ActorBoundingType::ACTOR_BOUNDING_CIRCLE):

		size = params_actor_bounding_.circle_radius;
		inflation = 2.15f * size;
		break;

	/* ACTOR_BOUNDING_ELLIPSE */
	case(actor::ActorBoundingType::ACTOR_BOUNDING_ELLIPSE):

		size_x = params_actor_bounding_.ellipse.at(0);
		size_y = params_actor_bounding_.ellipse.at(1);
		size = std::min(size_x, size_y);	// broader workspace
		// size = std::max(size_x, size_y); // problematic for costmap - really narrow workspace
		inflation = 2.15f * size;
		break;

	/* ACTOR_NO_BOUNDING */
	case(actor::ActorBoundingType::ACTOR_NO_BOUNDING):

		inflation = 0.01f;
		break;

	/* UNKNOWN id */
	default:

		return;
		break;

	}

	// update structure value
	params_actor_bounding_.inflation_radius = inflation;
	nh_ptr->setParam(ns_ + actor_ns_prefix_ + "inflation/inflation_radius", inflation);

}

// ------------------------------------------------------------------- //

void ParamLoader::calculateActorFootprint(const std::shared_ptr<ros::NodeHandle> nh_ptr) {

	// parameter
	XmlRpc::XmlRpcValue footprint;

	// evaluate bounding type
	switch (params_actor_bounding_.bounding_type) {

		case(actor::ActorBoundingType::ACTOR_BOUNDING_BOX):

				// define 4 corner points
				footprint.setSize(4);
				// upper-left
				footprint[0].setSize(2);
				footprint[0][0] = +params_actor_bounding_.box_size.at(0) * 0.5;
				footprint[0][1] = +params_actor_bounding_.box_size.at(1) * 0.5;
				// lower-left
				footprint[1].setSize(2);
				footprint[1][0] = -params_actor_bounding_.box_size.at(0) * 0.5;
				footprint[1][1] = +params_actor_bounding_.box_size.at(1) * 0.5;
				// lower-right
				footprint[2].setSize(2);
				footprint[2][0] = -params_actor_bounding_.box_size.at(0) * 0.5;
				footprint[2][1] = -params_actor_bounding_.box_size.at(1) * 0.5;
				// upper-right
				footprint[3].setSize(2);
				footprint[3][0] = +params_actor_bounding_.box_size.at(0) * 0.5;
				footprint[3][1] = -params_actor_bounding_.box_size.at(1) * 0.5;
				// publish
				nh_ptr->setParam(ns_ + actor_ns_prefix_ + "inflation/footprint", footprint);
				break;

		case(actor::ActorBoundingType::ACTOR_BOUNDING_ELLIPSE):

				// define 12 points around the center (0.0, 0.0)
				footprint.setSize(12);

				// list of angles
				std::vector<double> angles;
				double diff = 0.523599; // 30 degrees
				double alpha = 0.0;
				for ( size_t i = 0; i <= 6; i++ ) {
					angles.push_back(i * (+diff));
				}
				for ( size_t i = 5; i >= 1; i-- ) { // 0 was already added
					angles.push_back(i * (-diff));
				}

//				std::cout << "calculateActorFootprint | ELLIPSE | angles" << std::endl;
				for ( size_t i = 0; i < angles.size(); i++ ) {
//					std::cout << "\t\t" << i << " " << angles.at(i) << std::endl;
				}

//				if ( angles.size() != 12 ) {
//					std::cout << "ERROR! ANGLES SIZE IS: " << angles.size() << " instead of 12!" << std::endl;
//				}
//				std::cout << "FOOTPRINT:" << std::endl;
				// Parametric equation of an ellipse
				// x = a * cos(t) + x_center_shift
				// y = b * sin(t) + y_center_shift
				for ( size_t i = 0; i < angles.size(); i++ ) {

					// calculate points in the actor reference system
					double x = -params_actor_bounding_.ellipse.at(2) + 	// `x_center_shift`; see Enums.h note on sign
							   (params_actor_bounding_.ellipse.at(0) * 	// `a` / semi-major
							   cos(angles.at(i)));
					double y = -params_actor_bounding_.ellipse.at(3) + 	// `y_center_shift`; see Enums.h note on sign
							   (params_actor_bounding_.ellipse.at(1) * 	// `b` / semi-minor
							   sin(angles.at(i)));

					// convert points to the world reference system
					// const double ACTOR_REF_SYSTEM_ROTATION = 1.5708;
					double x_prim = x * cos(-1.5708) - y * sin(-1.5708);
					double y_prim = x * sin(-1.5708) + y * cos(-1.5708);

					// update the footprint contents
					footprint[i][0] = x_prim;
					footprint[i][1] = y_prim;

//					std::cout << "\tx = " << x << "\t y = " << y << std::endl;

				}

				// publish
				nh_ptr->setParam(ns_ + actor_ns_prefix_ + "inflation/footprint", footprint);
				break;

	}

}

// ------------------------------------------------------------------- //

ParamLoader::~ParamLoader() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
