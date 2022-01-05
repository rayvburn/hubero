#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_common/time.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

namespace hubero {

/**
 * @brief Helper struct to store animation information in map
 */
struct AnimationData {
    std::function<void(void)> handler;
    Time duration;
};

/**
 * @brief Class to provide human animation control from @ref Actor class level.
 * @details It must be defined by a human plugin of a particular simulator software.
 */
class AnimationControlBase {
public:
    AnimationControlBase(): initialized_(false), anim_finished_(true) {}

    void addAnimationHandler(
        AnimationType anim_type,
        std::function<void(void)> anim_handler,
        const Time& duration = Time(std::numeric_limits<double>::infinity())
    ) {
        AnimationData anim_data {};
        anim_data.handler = std::move(anim_handler);
        anim_data.duration = duration;
        map_animation_handlers_.insert({anim_type, anim_data});
        initialized_ = true;
    }

    /**
     * @brief Enables animation given by @ref anim_type
     */
    virtual bool start(AnimationType anim_type, Time time_current) {
        auto it = map_animation_handlers_.find(anim_type);
        if (it != map_animation_handlers_.end()) {
            if (it->second.handler == nullptr) {
                HUBERO_LOG("[AnimationControlBase] Cannot start animation since its handler was not defined\r\n");
                return false;
            }
            anim_finished_ = false;
            anim_active_ = it->first;

            time_begin_ = time_current;
            time_finish_ = Time::computeTimestamp(time_begin_, it->second.duration);

            it->second.handler();
            return true;
        }
        HUBERO_LOG("[AnimationControlBase] Cannot start animation since it was not defined\r\n");
        return false;
    }

    /**
     * @brief Additional method that allows to modify pose for specific animation by hand
     * @details Typically this method is empty since all animation operations are managed by a particular handler
     */
    virtual void adjustPose(Pose3& pose, const Time& time_current) const {}

    inline bool isInitialized() const {
		return initialized_;
	}

    inline bool isFinished() const {
        return anim_finished_;
    }

    inline AnimationType getActiveAnimation() const {
        return anim_active_;
    }

protected:
    /// True if at least 1 animation handler was added
    bool initialized_;
    bool anim_finished_;
    AnimationType anim_active_;

    Time time_begin_;
    Time time_finish_;

    std::map<AnimationType, AnimationData> map_animation_handlers_;
};

} // namespace hubero