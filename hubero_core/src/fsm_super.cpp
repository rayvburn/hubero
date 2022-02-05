#include <hubero_core/fsm/fsm_super.h>
#include <hubero_common/logger.h>

namespace hubero {

FsmSuper::FsmSuper(State state_init): fsm(state_init), FsmEssentials("FsmSuper") {

}

bool FsmSuper::anotherTaskRequested(const EventFsmSuper& event, const TaskPredicates& task_self) {
	std::vector<bool> tasks_requested;
	tasks_requested.push_back(event.follow_object.isPending());
	tasks_requested.push_back(event.lie_down.isPending());
	tasks_requested.push_back(event.move_around.isPending());
	tasks_requested.push_back(event.move_to_goal.isPending());
	tasks_requested.push_back(event.run.isPending());
	tasks_requested.push_back(event.sit_down.isPending());
	tasks_requested.push_back(event.stand.isPending());
	tasks_requested.push_back(event.talk.isPending());
	tasks_requested.push_back(event.teleop.isPending());
	int tasks_requested_num = std::count(tasks_requested.cbegin(), tasks_requested.cend(), true);
	// return true if there is another requested task
	return (tasks_requested_num - static_cast<int>(task_self.isPending())) > 0;
}

void FsmSuper::transHandlerStand2MoveToGoal(const EventFsmSuper& event) {
	logTransition("STAND", "MOVE TO GOAL", event);
	transitionHandler(State::STAND, State::MOVE_TO_GOAL);
}

void FsmSuper::transHandlerStand2MoveAround(const EventFsmSuper& event) {
	logTransition("STAND", "MOVE AROUND", event);
	transitionHandler(State::STAND, State::MOVE_AROUND);
}

void FsmSuper::transHandlerStand2FollowObject(const EventFsmSuper& event) {
	logTransition("STAND", "FOLLOW OBJECT", event);
	transitionHandler(State::STAND, State::FOLLOW_OBJECT);
}

void FsmSuper::transHandlerStand2LieDown(const EventFsmSuper& event) {
	logTransition("STAND", "LIE DOWN", event);
	transitionHandler(State::STAND, State::LIE_DOWN);
}

void FsmSuper::transHandlerStand2SitDown(const EventFsmSuper& event) {
	logTransition("STAND", "SIT DOWN", event);
	transitionHandler(State::STAND, State::SIT_DOWN);
}

void FsmSuper::transHandlerStand2Run(const EventFsmSuper& event) {
	logTransition("STAND", "RUN", event);
	transitionHandler(State::STAND, State::RUN);
}

void FsmSuper::transHandlerStand2Talk(const EventFsmSuper& event) {
	logTransition("STAND", "TALK", event);
	transitionHandler(State::STAND, State::TALK);
}

void FsmSuper::transHandlerStand2Teleop(const EventFsmSuper& event) {
	logTransition("STAND", "TELEOP", event);
	transitionHandler(State::STAND, State::TELEOP);
}

void FsmSuper::transHandlerMoveToGoal2Stand(const EventFsmSuper& event) {
	logTransition("MOVE TO GOAL", "STAND", event);
	transitionHandler(State::MOVE_TO_GOAL, State::STAND);
}

void FsmSuper::transHandlerMoveAround2Stand(const EventFsmSuper& event) {
	logTransition("MOVE AROUND", "STAND", event);
	transitionHandler(State::MOVE_AROUND, State::STAND);
}

void FsmSuper::transHandlerFollowObject2Stand(const EventFsmSuper& event) {
	logTransition("FOLLOW OBJECT", "STAND", event);
	transitionHandler(State::FOLLOW_OBJECT, State::STAND);
}

void FsmSuper::transHandlerLieDown2Stand(const EventFsmSuper& event) {
	logTransition("LIE DOWN", "STAND", event);
	transitionHandler(State::LIE_DOWN, State::STAND);
}

void FsmSuper::transHandlerSitDown2Stand(const EventFsmSuper& event) {
	logTransition("SIT DOWN", "STAND", event);
	transitionHandler(State::SIT_DOWN, State::STAND);
}

void FsmSuper::transHandlerRun2Stand(const EventFsmSuper& event) {
	logTransition("RUN", "STAND", event);
	transitionHandler(State::RUN, State::STAND);
}

void FsmSuper::transHandlerTalk2Stand(const EventFsmSuper& event) {
	logTransition("TALK", "STAND", event);
	transitionHandler(State::TALK, State::STAND);
}

void FsmSuper::transHandlerTeleop2Stand(const EventFsmSuper& event) {
	logTransition("TELEOP", "STAND", event);
	transitionHandler(State::TELEOP, State::STAND);
}

} // namespace hubero
