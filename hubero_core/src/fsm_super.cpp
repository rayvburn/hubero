#include <hubero_core/fsm_super.h>
#include <hubero_common/logger.h>

namespace hubero {

FsmSuper::FsmSuper(State state_init): fsm(state_init), logging_verbose_(false) {

}

void FsmSuper::setLoggingVerbosity(bool enable_verbose) {
	logging_verbose_ = enable_verbose;
}

void FsmSuper::setLoggerPreamble(const std::string& text) {
	logger_text_ = text;
}

void FsmSuper::logTransition(const EventFsmSuper& event) const {
	if (!logging_verbose_) {
		return;
	}
	HUBERO_LOG("[%s].[FsmSuper] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
}

void FsmSuper::transHandlerStand2MoveToGoal(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to MOVE TO GOAL\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2MoveAround(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to MOVE AROUND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2FollowObject(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to FOLLOW OBJECT\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2LieDown(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to LIE DOWN\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2Run(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to RUN\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2Talk(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to TALK\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerStand2Teleop(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from STAND to TELEOP\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerMoveToGoal2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from MOVE TO GOAL to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerMoveAround2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from MOVE AROUND to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerFollowObject2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from FOLLOW OBJECT to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerLieDown2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from LIE DOWN to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerRun2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from RUN to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerTalk2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from TALK to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

void FsmSuper::transHandlerTeleop2Stand(const EventFsmSuper& event) {
	HUBERO_LOG("[%s].[FsmSuper] transition from TELEOP to STAND\r\n", logger_text_.c_str());
	logTransition(event);
}

} // namespace hubero
