#include <hubero_core/fsm/fsm_super.h>
#include <hubero_common/logger.h>

namespace hubero {

FsmSuper::FsmSuper(State state_init): fsm(state_init), FsmEssentials("FsmSuper") {

}

bool FsmSuper::anotherTaskRequested(const EventFsmSuper& event, const TaskPredicates& task_self) {
	std::vector<bool> tasks_requested;
	tasks_requested.push_back(event.follow_object.requested);
	tasks_requested.push_back(event.lie_down.requested);
	tasks_requested.push_back(event.move_around.requested);
	tasks_requested.push_back(event.move_to_goal.requested);
	tasks_requested.push_back(event.run.requested);
	tasks_requested.push_back(event.sit_down.requested);
	tasks_requested.push_back(event.stand.requested);
	tasks_requested.push_back(event.talk.requested);
	tasks_requested.push_back(event.teleop.requested);
	int tasks_requested_num = std::count(tasks_requested.cbegin(), tasks_requested.cend(), true);
	// return true if there is another requested task
	return (tasks_requested_num - static_cast<int>(task_self.requested)) > 0;
}

void FsmSuper::transHandlerStand2MoveToGoal(const EventFsmSuper& event) {
	logTransition("STAND", "MOVE TO GOAL", event);
}

void FsmSuper::transHandlerStand2MoveAround(const EventFsmSuper& event) {
	logTransition("STAND", "MOVE AROUND", event);
}

void FsmSuper::transHandlerStand2FollowObject(const EventFsmSuper& event) {
	logTransition("STAND", "FOLLOW OBJECT", event);
}

void FsmSuper::transHandlerStand2LieDown(const EventFsmSuper& event) {
	logTransition("STAND", "LIE DOWN", event);
}

void FsmSuper::transHandlerStand2Run(const EventFsmSuper& event) {
	logTransition("STAND", "RUN", event);
}

void FsmSuper::transHandlerStand2Talk(const EventFsmSuper& event) {
	logTransition("STAND", "TALK", event);
}

void FsmSuper::transHandlerStand2Teleop(const EventFsmSuper& event) {
	logTransition("STAND", "TELEOP", event);
}

void FsmSuper::transHandlerMoveToGoal2Stand(const EventFsmSuper& event) {
	logTransition("MOVE TO GOAL", "STAND", event);
}

void FsmSuper::transHandlerMoveAround2Stand(const EventFsmSuper& event) {
	logTransition("MOVE AROUND", "STAND", event);
}

void FsmSuper::transHandlerFollowObject2Stand(const EventFsmSuper& event) {
	logTransition("FOLLOW OBJECT", "STAND", event);
}

void FsmSuper::transHandlerLieDown2Stand(const EventFsmSuper& event) {
	logTransition("LIE DOWN", "STAND", event);
}

void FsmSuper::transHandlerRun2Stand(const EventFsmSuper& event) {
	logTransition("RUN", "STAND", event);
}

void FsmSuper::transHandlerTalk2Stand(const EventFsmSuper& event) {
	logTransition("TALK", "STAND", event);
}

void FsmSuper::transHandlerTeleop2Stand(const EventFsmSuper& event) {
	logTransition("TELEOP", "STAND", event);
}

} // namespace hubero
