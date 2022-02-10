#pragma once

#include <string>
#include <hubero_common/typedefs.h>
#include <hubero_common/logger.h>
#include <hubero_interfaces/utils/model_geometry.h>

namespace hubero {

/**
 * @brief Provides unified world geometry representation for HuBeRo
 * @details ModelGeometry abstracts from specific simulator
 */
class WorldGeometryBase {
public:
	WorldGeometryBase(): initialized_(false) {}

	virtual void initialize(const std::string& world_frame_id) {
		frame_id_ = world_frame_id;
		initialized_ = true;
	}

	inline std::string getFrame() const {
		return frame_id_;
	}

	virtual ModelGeometry getModel(const std::string& name) const {
		if (!isInitialized()) {
			HUBERO_LOG("[WorldGeometryBase] 'getModel' call could not be processed due to lack of initialization\r\n");
			return ModelGeometry();
		}
		return ModelGeometry(name, getFrame());
	}

	inline bool isInitialized() const {
        return initialized_;
    }

protected:
	bool initialized_;
	std::string frame_id_;
};

} // namespace hubero
