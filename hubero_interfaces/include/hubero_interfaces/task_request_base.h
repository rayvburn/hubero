#pragma once

#include <hubero_common/defines.h>
#include <hubero_interfaces/utils/task_base.h>

#include <memory>
#include <utility>

namespace hubero {

/**
 * @brief This class acts as a generic interface to request Actor's tasks.
 */
class TaskRequestBase {
public:
    TaskRequestBase(): initialized_(false) {};

    /**
     * @brief Adds certain task so it can be called via some kind of class derived from this interface class
     */
    void addTask(TaskType task, std::shared_ptr<TaskBase> task_ptr) {
        tasks_map_.insert({task, task_ptr});
        initialized_ = true;
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

protected:
    /// True if at least 1 task was added
    bool initialized_;
    std::map<TaskType, std::shared_ptr<TaskBase>> tasks_map_;
};

} // namespace hubero
