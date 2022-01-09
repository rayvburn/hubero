#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_interfaces/utils/task_base.h>

#include <memory>
#include <string>
#include <utility>

namespace hubero {

/**
 * @brief This class acts as a generic interface to request Actor's tasks.
 *
 * @details This class must be initialized by @ref hubero::Actor that adds specific tasks that can be externally
 * requested via @ref hubero::TaskRequestBase API.
 */
class TaskRequestBase {
public:
    TaskRequestBase(): initialized_(false) {
        TaskRequestBase::task_names_map_ = {
            {"stand", TaskType::TASK_STAND},
            {"move_to_goal", TaskType::TASK_MOVE_TO_GOAL},
            {"move_around", TaskType::TASK_MOVE_AROUND},
            {"lie_down", TaskType::TASK_LIE_DOWN},
            {"sit_down", TaskType::TASK_SIT_DOWN},
            {"follow_object", TaskType::TASK_FOLLOW_OBJECT},
            {"teleop", TaskType::TASK_TELEOP},
            {"run", TaskType::TASK_RUN},
            {"talk", TaskType::TASK_TALK}
        };
    }

    /**
     * @brief Adds certain task so it can be called via some kind of class derived from this interface class
     */
    void addTask(TaskType task, std::shared_ptr<TaskBase> task_ptr) {
        tasks_map_.insert({task, task_ptr});
        // to treat class as initialized, at least 1 task must be added and base class' ctor must be called
        initialized_ = !task_names_map_.empty();
    }

    template <typename... Args>
    bool request(TaskType task, Args... task_args) {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot request task since its class was not defined\r\n");
                return false;
            }
            return it->second->request(std::forward<Args>(task_args)...);
        }
        HUBERO_LOG("[TaskRequestBase] Cannot request task since it was not defined\r\n");
        return false;
    }

    template <typename... Args>
    bool request(std::string task_name, Args... task_args) {
        return request(TaskRequestBase::getTaskType(task_name), std::forward<Args>(task_args)...);
    }

    bool abort(TaskType task) {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot abort task since its class was not defined\r\n");
                return false;
            }
            return it->second->abort();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot abort task since it was not defined\r\n");
        return false;
    }

    bool isRequested(TaskType task) const {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot check if task was requested since its class was not defined\r\n");
                return false;
            }
            return it->second->isRequested();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot check if task was requested since it was not defined\r\n");
        return false;
    }

    inline bool isInitialized() const {
        return initialized_;
    }

    /**
     * @brief Searches for @ref task_name in the @ref task_names_map_
     */
    static TaskType getTaskType(const std::string& task_name) {
        auto it = TaskRequestBase::task_names_map_.find(task_name);
        if (it == TaskRequestBase::task_names_map_.end()) {
            return TaskType::TASK_UNDEFINED;
        }
        return it->second;
    }

protected:
    /// True if at least 1 task was added
    bool initialized_;

    /// Binds TaskTypes to actual task classes that inherit TaskBase
    std::map<TaskType, std::shared_ptr<TaskBase>> tasks_map_;

    /// Binds task names (strings) to TaskTypes
    static std::map<std::string, TaskType> task_names_map_;
};

} // namespace hubero
