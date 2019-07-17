/*
 * ShiftRegister.h
 *
 *  Created on: Jul 17, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_SHIFTREGISTER_H_
#define INCLUDE_SFM_CORE_SHIFTREGISTER_H_

#include <vector>
#include <cstddef> 		// size_t
#include <algorithm>	// std::rotate

namespace sfm {
namespace core {

template <typename T>
class ShiftRegister {
public:

	/**
	 * @brief Parametrized constructor
	 * @param length - maximum size of the vector of values
	 */
	ShiftRegister(const size_t &length): capacity_(length) { }

	/**
	 * @brief Checks current size of the register.
	 * @return True if number of elements in vector is equal 0.
	 */
	bool isEmpty() const {
		if ( v_.size() == 0 ) {
			return (true);
		}
		return (false);
	}

	/**
	 * @brief Updates shift register, deletes the oldest value if number of elements is equal to register's
	 * capacity.
	 * @param value - value register will be filled with
	 * @return True when the oldest element was discarded, false when shift register is not full yet.
	 */
	bool update(const T &value) {

		if ( v_.size() < capacity_ ) {

			/* Some more elements can be loaded into register */

			// add a new element
			v_.push_back(value);

			return (false);

		} else {

			/* Register is full */

			// rotate left
			std::rotate(v_.begin(), v_.begin() + 1, v_.end());

			// update the last element (the newest)
			v_.at(capacity_ - 1) = value;

			return (true);

		}

	}

	/**
	 * @brief Clears register's content.
	 */
	void clear() {
		v_.clear();
	}

	/**
	 * @brief Calculates the average value based on vector content.
	 * @note This does not have any sense when class type is literal.
	 * @return
	 */
	T getAverage() const {

		T avg = T();
		for ( size_t i = 0; i < v_.size(); i++ ) {
			avg += v_.at(i);
		}
		avg /= v_.size();
		return (avg);

	}

	/**
	 * @brief Destructor
	 */
	virtual ~ShiftRegister() { }

private:

	/**
	 * @brief Maximum length of the vector. Set in constructor.
	 */
	size_t capacity_;

	/**
	 * @brief Container which stores all values.
	 */
	std::vector<T> v_;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_SHIFTREGISTER_H_ */
