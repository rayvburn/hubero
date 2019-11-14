/*
 * Regions.h
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_REGIONS_H_
#define INCLUDE_FUZZ_REGIONS_H_

namespace fuzz {

/* ------------- Input variables ------------- */

/// \brief Human-readable environment state definition
typedef enum {
	LOCATION_RIGHT = 0u,
	LOCATION_LEFT,
	LOCATION_FRONT,
	LOCATION_BACK,
	LOCATION_UNKNOWN
} FuzzLocation;

/// \brief TODO
typedef enum {
	FUZZ_DIR_EQUAL = 0u,//!< FUZZ_DIR_EQUAL
	FUZZ_DIR_OPPOSITE,  //!< FUZZ_DIR_OPPOSITE
	FUZZ_DIR_PERP_OUT,  //!< FUZZ_DIR_PERP_OUT
	FUZZ_DIR_PERP_CROSS,//!< FUZZ_DIR_PERP_CROSS
	FUZZ_DIR_UNKNOWN    //!< FUZZ_DIR_UNKNOWN
} FuzzDirection;

/* ------------- Output variable ------------- */

/// \brief Output behaviors enumerator.
/// \note The enum value must be equal to the upper bound of the behavior range.
typedef enum {
	FUZZ_BEH_NONE = 0,			   //!< FUZZ_BEH_NONE
	FUZZ_BEH_TURN_LEFT,            //!< FUZZ_BEH_TURN_LEFT
	FUZZ_BEH_TURN_LEFT_ACCELERATE, //!< FUZZ_BEH_TURN_LEFT_ACCELERATE
	FUZZ_BEH_GO_ALONG,             //!< FUZZ_BEH_GO_ALONG
	FUZZ_BEH_ACCELERATE,           //!< FUZZ_BEH_ACCELERATE
	FUZZ_BEH_TURN_RIGHT_ACCELERATE,//!< FUZZ_BEH_TURN_RIGHT_ACCELERATE
	FUZZ_BEH_TURN_RIGHT,           //!< FUZZ_BEH_TURN_RIGHT
	FUZZ_BEH_TURN_RIGHT_DECELERATE,//!< FUZZ_BEH_TURN_RIGHT_DECELERATE
	FUZZ_BEH_STOP,                 //!< FUZZ_BEH_STOP
	FUZZ_BEH_DECELERATE            //!< FUZZ_BEH_DECELERATE
} FuzzBehavior;

} /* namespace fuzz */

#endif /* INCLUDE_FUZZ_REGIONS_H_ */
