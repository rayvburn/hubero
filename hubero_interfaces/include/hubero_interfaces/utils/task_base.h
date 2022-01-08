#pragma once

#include <hubero_common/defines.h>

#include <functional>
#include <map>
#include <type_traits>

namespace hubero {

/**
 * @brief Provides interface class for various task definitions that can be requested from HuBeRo actors
 */
class TaskBase {
public:
    static constexpr int TASK_ARGS_NUM_DEFAULT = 127;

    TaskBase(TaskType task):
        task_type_(task),
        requested_(false),
        aborted_(false),
        finished_(true),
        task_args_num_(TASK_ARGS_NUM_DEFAULT) {}


    static bool addBasicBehaviourHandler(BasicBehaviourType behaviour_type, std::function<void(void)> handler) {
        basic_behaviour_handlers_.insert({behaviour_type, std::move(handler)});
    }

    /**
     * @brief Must be called at the start of each @ref request in derived class
     */
    template <typename... Args>
    bool request(Args... task_args) {
        if (sizeof...(task_args) != task_args_num_) {
            aborted_ = false;
            finished_ = true;
            requested_ = false;
            return false;
        }
        aborted_ = false;
        finished_ = false;
        requested_ = true;
    }

    /**
     * @brief Must be called at the end of each @ref abort in dervied class
     */
    bool abort() {
        bool actual_abort_call = requested_;
        aborted_ = true;
        finished_ = true;
        requested_ = false;
        return actual_abort_call;
    }

    TaskType getTaskType() const {
        return task_type_;
    }

    bool isRequested() const {
        return requested_;
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
    bool aborted_;
    bool finished_;

    /**
     * @brief How many arguments are required to be passed to @ref request method - this must be redefined by a specific Task
     * @details https://stackoverflow.com/questions/36797770/get-function-parameters-count
     */
    size_t task_args_num_;

    static std::map<BasicBehaviourType, std::function<void(void)>> basic_behaviour_handlers_;
}; // class TaskBase

} // namespace hubero
