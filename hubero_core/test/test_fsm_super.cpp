#include <gtest/gtest.h>
#include <hubero_core/fsm/fsm_super.h>

#include <iostream>

using namespace hubero;

/**
 * It does not matter which state is checked since all share the same terminal and transition conditions methods
 */

/// Evaluate initial state of the FSM passed to ctor
TEST(HuberoFsm, initialState) {
	FsmSuper fsm1(FsmSuper::State::TELEOP);
	ASSERT_EQ(fsm1.current_state(), FsmSuper::State::TELEOP);

	FsmSuper fsm2(FsmSuper::State::LIE_DOWN);
	ASSERT_EQ(fsm2.current_state(), FsmSuper::State::LIE_DOWN);
}

/// Normal execution of the task - safely finished
TEST(HuberoFsm, normal) {
	FsmSuper fsm;
	fsm.setLoggerPreamble("testNormal");
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	// change state and keep operating in it
	EventFsmSuper event {};
	event.stand.finished = true;
	event.move_to_goal.requested = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	event.stand.requested = false;
	event.stand.finished = false;
	event.move_to_goal.requested = false;

	for (unsigned int i = 0; i < 25; i++) {
		fsm.process_event(event);
		ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);
	}

	// finish
	event.move_to_goal.requested = false;
	event.move_to_goal.finished = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);
}

/// Task aborted during its execution
TEST(HuberoFsm, aborted) {
	FsmSuper fsm;
	fsm.setLoggerPreamble("testAbort");
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	EventFsmSuper event {};

	event.move_to_goal.requested = true;
	event.move_to_goal.finished = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	event.move_to_goal.requested = false;
	event.move_to_goal.aborted = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);
}

/// Requested another task during first execution
TEST(HuberoFsm, interrupted) {
	FsmSuper fsm;
	fsm.setLoggerPreamble("testInterrupt");
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	EventFsmSuper event {};

	// try to interrupt with another request
	event.move_to_goal.requested = true;
	event.move_to_goal.aborted = false;
	event.move_to_goal.finished = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	// already switched to MOVE_TO_GOAL, requested flag is erased
	event.move_to_goal.requested = false;
	event.move_around.requested = true;
	fsm.process_event(event);
	// another task requested - first switch to STAND, next switch to the new task
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_AROUND);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
