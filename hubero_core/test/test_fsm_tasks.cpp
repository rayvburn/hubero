#include <gtest/gtest.h>

#include <hubero_core/fsm/fsm_basic.h>
#include <hubero_core/fsm/fsm_follow_object.h>
#include <hubero_core/fsm/fsm_lie_down.h>
#include <hubero_core/fsm/fsm_move_around.h>
#include <hubero_core/fsm/fsm_sit_down.h>
#include <hubero_core/fsm/fsm_talk.h>

#include <hubero_interfaces/utils/task_base.h>

#include <hubero_core/tasks/task_follow_object.h>
#include <hubero_core/tasks/task_lie_down.h>
#include <hubero_core/tasks/task_move_around.h>
#include <hubero_core/tasks/task_sit_down.h>
#include <hubero_core/tasks/task_talk.h>

using namespace hubero;

/*
 * Tests related to internal FSMs of tasks
 */

/// Basic FSM
TEST(HuberoFsmTasks, fsmBasicNormal) {
	TaskBase task(TaskType::TASK_MOVE_TO_GOAL);
	FsmBasic fsm;

	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	// pending navigation task will not trigger active state
	task.request(Pose3(5.0, 5.0, 0.0, 0.0, 0.0, 0.0));

	EventFsmBasic e1(task, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	EventFsmBasic e2(task, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);
	// refresh
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);

	// navigation succeeded
	EventFsmBasic e3(task, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	// new goal received
	task.request(Pose3(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0));
	EventFsmBasic e4(task, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e4);

	EventFsmBasic e5(task, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);

	// navigation succeeded
	EventFsmBasic e6(task, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e6);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	// new goal that will be aborted
	task.request(Pose3(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0));
	EventFsmBasic e8(task, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e8);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);

	EventFsmBasic e9(task, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e9);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::ACTIVE);

	EventFsmBasic e10(task, TaskFeedbackType::TASK_FEEDBACK_ABORTED);
	fsm.process_event(e10);
	ASSERT_EQ(fsm.current_state(), FsmBasic::State::FINISHED);
}

TEST(HuberoFsmTasks, followObject) {
	auto task_ptr = std::make_shared<TaskFollowObject>();
	FsmFollowObject fsm;

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);
	const auto INITIAL_STATE = fsm.current_state();

	// request task first
	task_ptr->request("object");
	EventFsmFollowObject e1(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// activate task
	task_ptr->activate();
	EventFsmFollowObject e2(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// still no plan generated to followed object
	EventFsmFollowObject e3(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// got a plan
	EventFsmFollowObject e4(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e4);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// object still far away
	EventFsmFollowObject e5(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE, false);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// object approached close enough
	EventFsmFollowObject e6(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE, true);
	fsm.process_event(e6);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);

	// catched 'terminated' nav state
	EventFsmFollowObject e7(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED, true);
	fsm.process_event(e7);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);

	// object moved away, still no plan
	EventFsmFollowObject e8(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING, false);
	fsm.process_event(e8);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// keep following with a valid plan in navigation module
	EventFsmFollowObject e9(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE, false);
	fsm.process_event(e9);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// approached object again, this time goal was reached perfectly (succeeded)
	EventFsmFollowObject e10(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED, true);
	fsm.process_event(e10);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);

	// let's say, goal moved away again and we have a valid plan
	EventFsmFollowObject e11(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE, false);
	fsm.process_event(e11);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	// terminate task and check state that next execution will start with
	task_ptr->terminate();
	EventFsmFollowObject e12(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e12);
	ASSERT_EQ(fsm.current_state(), INITIAL_STATE);

	// check what happens when task terminated in WAITING FOR MOVEMENT
	task_ptr->request("object2");
	task_ptr->activate();
	EventFsmFollowObject e13(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e13);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::MOVING_TO_GOAL);

	EventFsmFollowObject e14(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	e14.setObjectNearby(true);
	fsm.process_event(e14);
	ASSERT_EQ(fsm.current_state(), FsmFollowObject::State::WAITING_FOR_MOVEMENT);

	// make sure that termination while in WAITING FOR MOVEMENT switches back to MOVING_TO_GOAL
	task_ptr->terminate();
	EventFsmFollowObject e15(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e15);
	ASSERT_EQ(fsm.current_state(), INITIAL_STATE);
}

TEST(HuberoFsmTasks, lieDown) {
	auto task_ptr = std::make_shared<TaskLieDown>();
	FsmLieDown fsm;

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);
	const auto INITIAL_STATE = fsm.current_state();

	// moving to goal
	task_ptr->request(Vector3(2.0, 3.0, 0.0), IGN_PI_2);
	task_ptr->activate();

	// no plan for navigation created so far
	EventFsmLieDown e1(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// got a plan to goal
	EventFsmLieDown e2(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// for some reason, this goal is aborted
	task_ptr->abort();
	EventFsmLieDown e3(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// start again, got a request and found a plan
	task_ptr->request(Vector3(-2.0, -3.0, 0.0), IGN_PI_2);
	EventFsmLieDown e4(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e4);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::MOVING_TO_GOAL);

	// TODO: stages: lying down, lying, standing up should not be abortable!
	// lie down pose reached - start lying down
	EventFsmLieDown e5(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING_DOWN);

	// checking if finishing navigation part does any change (we don't expect it to)
	EventFsmLieDown e6(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e6);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING_DOWN);

	// still  lying down
	EventFsmLieDown e7(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e7);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING_DOWN);

	// lied down
	EventFsmLieDown e8(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e8.setLiedDown(true);
	fsm.process_event(e8);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING);

	// still lying
	EventFsmLieDown e9(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e9.setLiedDown(true);
	fsm.process_event(e9);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::LYING);

	// task abort request received - standing up
	task_ptr->abort();
	EventFsmLieDown e10(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e10.setLiedDown(true);
	fsm.process_event(e10);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING_UP);

	// still standing up
	EventFsmLieDown e11(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e11);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING_UP);

	// finished standing up
	EventFsmLieDown e12(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e12.setStoodUp(true);
	fsm.process_event(e12);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING);

	// being stood up, lack of stood up flag should not do anything bad
	EventFsmLieDown e13(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e13);
	ASSERT_EQ(fsm.current_state(), FsmLieDown::State::STANDING);

	// after task finish - FSM must be in INITIAL_STATE
	task_ptr->finish();
	EventFsmLieDown e14(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e14);
	ASSERT_EQ(fsm.current_state(), INITIAL_STATE);
}

TEST(HuberoFsmTasks, moveAround) {
	auto task_ptr = std::make_shared<TaskMoveAround>();
	FsmMoveAround fsm;

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// task requested, 'navigation' not planned yet
	task_ptr->request();
	EventFsmMoveAround e1(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// got a valid navigation plan
	EventFsmMoveAround e2(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);

	// keep going towards goal
	EventFsmMoveAround e3(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);

	// first goal reached
	EventFsmMoveAround e4(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e4);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// looking for a goal, but navigation task seems to be succeeded
	EventFsmMoveAround e5(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// keep looking for a new goal, but now navigation task has gently finished
	EventFsmMoveAround e6(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e6);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);

	// found new goal
	EventFsmMoveAround e7(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e7);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::MOVING_TO_GOAL);

	// finish task and let's say, we start from choosing goal again
	task_ptr->finish();
	EventFsmMoveAround e8(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e8);
	ASSERT_EQ(fsm.current_state(), FsmMoveAround::State::CHOOSING_GOAL);
}

TEST(HuberoFsmTasks, sitDown) {
	auto task_ptr = std::make_shared<TaskSitDown>();
	FsmSitDown fsm;

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);
	const auto INITIAL_STATE = fsm.current_state();

	// task requested, but no plan acquired yet
	task_ptr->request(Vector3(-3.0, 2.0, 0.0), 0.0);
	EventFsmSitDown e1(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);

	// navigation plan ready
	task_ptr->activate();
	EventFsmSitDown e2(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e2);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::MOVING_TO_GOAL);

	// reached sit down place
	EventFsmSitDown e3(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING_DOWN);

	// still sitting down
	fsm.process_event(e3);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING_DOWN);

	// navigation state erased to default
	EventFsmSitDown e4(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e4);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING_DOWN);

	// sat down
	EventFsmSitDown e5(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e5.setSatDown(true);
	fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING);

	// still sitting, even sat down flag erased
	EventFsmSitDown e6(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e6);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::SITTING);

	// task requested to abort, stand up
	task_ptr->abort();
	EventFsmSitDown e7(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e7);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING_UP);

	// still standing up
	EventFsmSitDown e8(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e8);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING_UP);

	// stood up
	EventFsmSitDown e9(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	e9.setStoodUp(true);
	fsm.process_event(e9);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING);

	// 'stood up' flag erased, but standing still
	EventFsmSitDown e10(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e10);
	ASSERT_EQ(fsm.current_state(), FsmSitDown::State::STANDING);

	// task finished
	task_ptr->finish();
	EventFsmSitDown e11(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e11);
	EXPECT_EQ(fsm.current_state(), INITIAL_STATE);

	// task terminated
	task_ptr->terminate();
	EventFsmSitDown e12(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e12);
	EXPECT_EQ(fsm.current_state(), INITIAL_STATE);
}

TEST(HuberoFsmTasks, talk) {
	auto task_ptr = std::make_shared<TaskTalk>();
	FsmTalk fsm;

	// initial state
	ASSERT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// task request
	task_ptr->request(Pose3(2.0, 4.0, 0.0, 0.0, 0.0, 0.0));
	EventFsmTalk e1(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e1);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// task activation
	task_ptr->activate();
	EventFsmTalk e2(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_PENDING);
	fsm.process_event(e2);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// trying to reach the goal pose
	EventFsmTalk e3(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_ACTIVE);
	fsm.process_event(e3);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	fsm.process_event(e3);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// goal pose reached
	EventFsmTalk e4(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e4);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::TALKING);

	// still talking
	EventFsmTalk e5(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e5);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::TALKING);

	// task finished
	task_ptr->finish();
	EventFsmTalk e6(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);
	fsm.process_event(e6);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);

	// this should not affect
	EventFsmTalk e7(*task_ptr, TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
	fsm.process_event(e7);
	EXPECT_EQ(fsm.current_state(), FsmTalk::State::MOVING_TO_GOAL);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
