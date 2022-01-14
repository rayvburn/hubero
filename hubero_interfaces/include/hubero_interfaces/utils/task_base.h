#pragma once

#include <hubero_common/defines.h>

#include <functional>
#include <map>
#include <type_traits>

namespace hubero {

// TODO: would look cleaner with C++17 'static constexpr'
/*
 * Workaround that allows to use static members in header-only library with C++11/14:
 * https://stackoverflow.com/questions/11709859/how-to-have-static-data-members-in-a-header-only-library
 */
template <typename Tkey, typename Tval>
struct _StaticMapBbHandlers {
    static std::map<Tkey, Tval> basic_behaviour_handlers_;
};

template <typename Tkey, typename Tval>
std::map<Tkey, Tval> _StaticMapBbHandlers<Tkey, Tval>::basic_behaviour_handlers_;

/**
 * @brief Provides interface class for various task definitions that can be requested from HuBeRo actors
 * @details Typical task lifecycle:
 *   - requested, not active
 *   - activation, requested flag erased
 *   - active
 *   - finished, active flag erased
 */
class TaskBase: protected _StaticMapBbHandlers<BasicBehaviourType, std::function<void(void)>> {
public:
    static constexpr int TASK_ARGS_NUM_DEFAULT = 0;

    TaskBase(TaskType task):
        task_type_(task),
        requested_(false),
        aborted_(false),
        finished_(false),
        feedback_type_(TASK_FEEDBACK_UNDEFINED),
        task_args_num_(TASK_ARGS_NUM_DEFAULT) {}


    static bool addBasicBehaviourHandler(BasicBehaviourType behaviour_type, std::function<void(void)> handler) {
        auto status = TaskBase::basic_behaviour_handlers_.insert({behaviour_type, std::move(handler)});
        return status.second;
    }

    /**
     * @brief Must be called at the start of each @ref request in derived class
     */
    template <typename... Args>
    bool request(Args... task_args) {
        if (sizeof...(task_args) != task_args_num_) {
            aborted_ = false;
            finished_ = false;
            requested_ = false;
            feedback_type_ = TASK_FEEDBACK_REJECTED;
            return false;
        }
        aborted_ = false;
        finished_ = false;
        requested_ = true;
        feedback_type_ = TASK_FEEDBACK_PENDING;
    }

    /**
     * @brief Must be called at the end of each @ref abort in dervied class
     */
    bool abort() {
        bool actual_abort_call = requested_;
        aborted_ = true;
        finished_ = false;
        requested_ = false;
        feedback_type_ = TASK_FEEDBACK_ABORTED;
        return actual_abort_call;
    }

    virtual inline void activate() {
        active_ = true;
        aborted_ = false;
        // leave finished_ as it is
        requested_ = false;
        feedback_type_ = TASK_FEEDBACK_ACTIVE;
    }

    virtual inline void terminate() {
        active_ = false;
        aborted_ = false;
        finished_ = false;
        feedback_type_ = TASK_FEEDBACK_SUCCEEDED;
    }

    TaskType getTaskType() const {
        return task_type_;
    }

    TaskFeedbackType getTaskFeedbackType() const {
        return feedback_type_;
    }

    bool isRequested() const {
        return requested_;
    }

    bool isActive() const {
        return active_;
    }

    bool isAborted() const {
        return aborted_;
    }

    bool isFinished() const {
        return finished_;
    }

    unsigned int getTaskArgsNumber() const {
        return task_args_num_;
    }

protected:
    /**
     * @brief Counts number of arguments of class method
     * @details This method should be used in constructor of specific task to define @ref task_args_num_
     * @note https://stackoverflow.com/questions/64312577/get-number-of-arguments-in-a-class-member-function
     */
    template <typename R, typename T, typename ... Types>
    size_t countArgumentsNum(R(T::*)(Types ...)) {
        return static_cast<size_t>(std::integral_constant<unsigned, sizeof ...(Types)>{}.value);
    }

    TaskType task_type_;

    bool requested_;
    bool active_;
    bool aborted_;
    bool finished_;

    TaskFeedbackType feedback_type_;

    /**
     * @brief How many arguments are required to be passed to @ref request method - this must be redefined by a specific Task
     * @details https://stackoverflow.com/questions/36797770/get-function-parameters-count
     */
    size_t task_args_num_;
}; // class TaskBase

} // namespace hubero
