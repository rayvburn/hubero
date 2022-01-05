#pragma once

#include <string>
#include <hubero_common/typedefs.h>
#include <hubero_interfaces/utils/model_geometry.h>

namespace hubero {

/**
 * @brief Provides unified world geometry representation for HuBeRo
 * @details ModelGeometry abstracts from specific simulator
 */
class WorldGeometryBase {
public:
	WorldGeometryBase(const std::string& world_frame_id): frame_id_(world_frame_id) {}

	inline std::string getFrame() {
		return frame_id_;
	}

	virtual ModelGeometry getModel(const std::string& name) {
		return ModelGeometry(name);
	}

protected:
	std::string frame_id_;
};

} // namespace hubero
