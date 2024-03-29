#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_interfaces/utils/task_base.h>

#include <memory>
#include <string>
#include <utility>

namespace hubero {

// TODO: would look cleaner with C++17
/*
 * Workaround that allows to use static members in header-only library with C++11/14:
 * https://stackoverflow.com/questions/11709859/how-to-have-static-data-members-in-a-header-only-library
 */
template <typename Tkey, typename Tval>
struct _StaticMapTaskNames {
    /// Binds task names (strings) to TaskTypes
    static std::map<Tkey, Tval> task_names_map_;
};

template <typename Tkey, typename Tval>
std::map<Tkey, Tval> _StaticMapTaskNames<Tkey, Tval>::task_names_map_ = {
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

/**
 * @brief This class acts as a generic interface to request Actor's tasks.
 *
 * @details This class must be initialized by @ref hubero::Actor that adds specific tasks that can be externally
 * requested via @ref hubero::TaskRequestBase API.
 */
class TaskRequestBase: protected _StaticMapTaskNames<std::string, TaskType> {
public:
    TaskRequestBase(): initialized_(false) {}

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

    /**
     * @brief Activates given task
     *
     * @note This may be useful while handling one task and the same task was requested to be executed with 
     * different objectives
     */
    void activate(TaskType task) {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot activate task since its class was not defined\r\n");
                return;
            }
            return it->second->activate();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot activate task since it was not defined\r\n");
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

    bool isActive(TaskType task) const {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot check if task is active since its class was not defined\r\n");
                return false;
            }
            return it->second->isActive();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot check if task is active since it was not defined\r\n");
        return false;
    }

    bool isAborted(TaskType task) const {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot check if task was aborted since its class was not defined\r\n");
                return false;
            }
            return it->second->isAborted();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot check if task was aborted since it was not defined\r\n");
        return false;
    }

    bool isFinished(TaskType task) const {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot check if task is finished since its class was not defined\r\n");
                return false;
            }
            return it->second->isFinished();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot check if task is finished since it was not defined\r\n");
        return false;
    }

    TaskFeedbackType getTaskFeedbackType(TaskType task) const {
        auto it = tasks_map_.find(task);
        if (it != tasks_map_.end()) {
            if (it->second == nullptr) {
                HUBERO_LOG("[TaskRequestBase] Cannot check task feedback since task's class was not defined\r\n");
                return TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;
            }
            return it->second->getTaskFeedbackType();
        }
        HUBERO_LOG("[TaskRequestBase] Cannot check task feedback since task was not defined\r\n");
        return TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;
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

    /**
     * @brief Searches for @ref task_type value in the @ref task_names_map_ and returns corresponding key from the map
     */
    static std::string getTaskName(const TaskType& task_type) {
        for (const auto& task_name_pair: TaskRequestBase::task_names_map_) {
            if (task_name_pair.second == task_type) {
                return task_name_pair.first;
            }
        }
        return std::string();
    }

protected:
    /// True if at least 1 task was added
    bool initialized_;

    /// Binds TaskTypes to actual task classes that inherit TaskBase
    std::map<TaskType, std::shared_ptr<TaskBase>> tasks_map_;
};

} // namespace hubero
