#include <gtest/gtest.h>

#include <hubero_core/tasks/fsm_basic.h>
#include <hubero_core/tasks/fsm_follow_object.h>
#include <hubero_core/tasks/fsm_lie_down.h>
#include <hubero_core/tasks/fsm_move_around.h>
#include <hubero_core/tasks/fsm_sit_down.h>
#include <hubero_core/tasks/fsm_talk.h>

using namespace hubero;

/// single state 'fsm'
TEST(HuberoFsmTasks, basic) {
	FsmBasic fsm;
	EventFsmBasic event {};
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);
}

TEST(HuberoFsmTasks, followObject) {
	FsmFollowObject fsm;
	EventFsmFollowObject event {};
	fsm.setLoggerPreamble("follow object FSM test");

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// object far away
	event.object_nearby = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// object in close proximity
	event.object_nearby = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);

	// object far away
	event.object_nearby = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);
}

TEST(HuberoFsmTasks, lieDown) {
	FsmLieDown fsm;
	EventFsmLieDown event {};
	fsm.setLoggerPreamble("lie down FSM test");

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// moving to goal
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// lying down
	event.goal_reached = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING_DOWN);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING_DOWN);

	// lying
	event.lied_down = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING);

	// standing up
	event.aborted = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING_UP);

	// stand
	event.stood_up = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING);
}

TEST(HuberoFsmTasks, moveAround) {
	FsmMoveAround fsm;
	EventFsmMoveAround event {};
	fsm.setLoggerPreamble("move around FSM test");

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// moving to goal not interrupted
	event.goal_selected = true;
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);

	// first goal reached, but selected flag was not erased
	event.goal_selected = true;
	event.goal_reached = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);
	// selected flag also erased
	event.goal_selected = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// finally found new goal but reached flag not set to false
	event.goal_selected = true;
	event.goal_reached = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// indicate that moving to goal is necessary
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);

	// still moving to goal
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);
}

TEST(HuberoFsmTasks, sitDown) {
	FsmSitDown fsm;
	EventFsmSitDown event {};
	fsm.setLoggerPreamble("sit down FSM test");

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);

	// moving to goal
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);

	// sitting down
	event.goal_reached = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING_DOWN);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING_DOWN);

	// sitting
	event.sat_down = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING);

	// standing up
	event.aborted = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING_UP);

	// stand
	event.stood_up = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING);
}

TEST(HuberoFsmTasks, talk) {
	FsmTalk fsm;
	EventFsmTalk event {};
	fsm.setLoggerPreamble("talk FSM test");

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// goal still not reached
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// goal reached
	event.goal_reached = true;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::TALKING);
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::TALKING);

	// goal has moved away
	event.goal_reached = false;
	fsm.process_event(event);
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
