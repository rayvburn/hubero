/*
 * TargetLot.h
 *
 *  Created on: Mar 4, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_TARGETLOT_H_
#define INCLUDE_ACTOR_CORE_TARGETLOT_H_

#include <ignition/math/Vector3.hh>

namespace actor {
namespace core {

/// @brief The `T` type must have an `==` operator defined
template<typename T>
class TargetLot {

public:

//	/// @brief
//	TargetLot(): safe_defined_(false), raw_defined_(false) {}

	/// @brief
	TargetLot(const T &target_raw = T()): safe_defined_(false), raw_defined_(true), raw_(target_raw) {}

	/// @brief
	TargetLot(const T &target_raw, const T &target_safe): raw_defined_(true),raw_(target_raw),
			safe_defined_(true), safe_(target_safe) {}

	/// @brief
	/// @param target
	void setSafe(const T &target) {

		// investigate all possible permutations
		if ( !safe_defined_ && raw_defined_ ) {
			// `safe` element definition
			safe_defined_ = true;
		} else if ( safe_defined_ && !raw_defined_ ) {
			// `safe` element redefinition, `raw` not defined yet

		} else if ( safe_defined_ && raw_defined_ ) {
			// `safe` element redefinition, safe was defined previously, `raw` is a leftover from the last call
			raw_defined_ = false;
		} else if ( !safe_defined_ && !raw_defined_ ) {
			// none of elements was defined - this is probably the first iteration of the application
			safe_defined_ = true;
		}
		safe_ = target;

	}

	/// @brief
	/// @param target
	void setRaw(const T &target) {

		// investigate all possible permutations
		if ( !safe_defined_ && raw_defined_ ) {
			// `raw` element redefinition

		} else if ( safe_defined_ && !raw_defined_ ) {
			// `raw` element definition, safe was defined previously
			raw_defined_ = true;
		} else if ( safe_defined_ && raw_defined_ ) {
			// `raw` element redefinition, safe is a leftover from the last call
			safe_defined_ = false;
		} else if ( !safe_defined_ && !raw_defined_ ) {
			// none of elements was defined - this is probably the first iteration of the application
			raw_defined_ = true;
		}
		raw_ = target;

	}

	/// @brief
	/// @return
	bool areEqual() const {
		if ( raw_ == safe_ ) {
			return (true);
		}
		return (false);
	}

	/// @brief
	/// @return
	bool isSafeDefined() const 	{ return (safe_defined_); 	}
	/// @brief
	/// @return
	T getSafe() const 			{ return (safe_); 			}
	/// @brief
	/// @return
	T getRaw() const 			{ return (raw_); 			}
	/// @brief
	virtual ~TargetLot() 		{							}

private:

	/// @brief The safe point according to the static global costmap
	bool safe_defined_;
	/// @brief The actual target
	bool raw_defined_;

	/// @brief
	T safe_;
	/// @brief
	T raw_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_TARGETLOT_H_ */
