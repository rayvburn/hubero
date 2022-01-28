#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

struct EventFsmBasic: public TaskPredicates {
    std::string toString() const {
        return TaskPredicates::toString();
    }
};

class FsmBasic: public fsmlite::fsm<FsmBasic> {
public:
    FsmBasic(int state_init = 0): fsm(state_init) {}

protected:
    // NOTE: logTransition is not required here since no transitions will be performed
    // NOTE: FSM transition guards are not required too
    // NOTE: FSM transition handlers are not required too

    // NOTE: transition table is required by fsmlite (at least in a dummy form)
    using transition_table = table<>;

    friend class fsmlite::fsm<FsmBasic>;
};

} // namespace hubero
