#include <gtest/gtest.h>

#include <hubero_core/actor.h>

#include <memory>

#include <iostream>

using namespace hubero;

/**
 * It does not matter which state is checked since all share the same terminal and transition conditions methods,
 * @see FsmSuper guards
 */

/**
 * @brief Test fixture that wraps all supported tasks to evaluate operation of the highest level of the FSM
 */
class FsmSuperTasksTest: public ::testing::Test {
protected:
	FsmSuperTasksTest():
		task_follow_object_ptr(std::make_shared<TaskFollowObject>()),
		task_lie_down_ptr(std::make_shared<TaskLieDown>()),
		task_move_around_ptr(std::make_shared<TaskMoveAround>()),
		task_move_to_goal_ptr(std::make_shared<TaskMoveToGoal>()),
		task_run_ptr(std::make_shared<TaskRun>()),
		task_sit_down_ptr(std::make_shared<TaskSitDown>()),
		task_stand_ptr(std::make_shared<TaskStand>()),
		task_talk_ptr(std::make_shared<TaskTalk>()),
		task_teleop_ptr(std::make_shared<TaskTeleop>()),
		navigation_ptr(std::make_shared<NavigationBase>()) {}

	void SetUp() override {
		Actor::addFsmSuperTransitionHandlers(
			fsm,
			task_stand_ptr,
			task_move_to_goal_ptr,
			task_move_around_ptr,
			task_lie_down_ptr,
			task_sit_down_ptr,
			task_follow_object_ptr,
			task_teleop_ptr,
			task_run_ptr,
			task_talk_ptr,
			navigation_ptr
		);
	}

    EventFsmSuper composeEventFsm() {
        return EventFsmSuper(
            task_follow_object_ptr,
            task_lie_down_ptr,
            task_move_around_ptr,
            task_move_to_goal_ptr,
            task_run_ptr,
            task_sit_down_ptr,
            task_stand_ptr,
            task_talk_ptr,
            task_teleop_ptr
        );
    }

    TaskPredicates predicatesTaskFinished() const {
        TaskPredicates p(false, false, false, true);
    }

    TaskPredicates predicatesTaskAborted() const {
        TaskPredicates p(false, false, true, false);
    }

    FsmSuper fsm;
	std::shared_ptr<TaskFollowObject> task_follow_object_ptr;
	std::shared_ptr<TaskLieDown> task_lie_down_ptr;
	std::shared_ptr<TaskMoveAround> task_move_around_ptr;
	std::shared_ptr<TaskMoveToGoal> task_move_to_goal_ptr;
	std::shared_ptr<TaskRun> task_run_ptr;
	std::shared_ptr<TaskSitDown> task_sit_down_ptr;
	std::shared_ptr<TaskStand> task_stand_ptr;
	std::shared_ptr<TaskTalk> task_talk_ptr;
	std::shared_ptr<TaskTeleop> task_teleop_ptr;

	std::shared_ptr<NavigationBase> navigation_ptr;
};

TEST_F(FsmSuperTasksTest, initial) {
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);
}

// try to move to goal requested twice, second execution is cancelled
TEST_F(FsmSuperTasksTest, moveToGoalNormalAndAborted) {
    // goal requested
	task_move_to_goal_ptr->request(Pose3(5.0, 5.0, 0.0, 0.0, 0.0, 0.0));
    auto e1 = composeEventFsm();
    ASSERT_TRUE(e1.move_to_goal.isPending());
	ASSERT_FALSE(e1.move_to_goal.isActive());
	ASSERT_FALSE(e1.move_to_goal.isAborted());
	ASSERT_FALSE(e1.move_to_goal.isSucceeded());
	ASSERT_FALSE(e1.move_to_goal.isEnded());
    fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	// since state switched, task should be activated with transition handler
	auto e2 = composeEventFsm();
	ASSERT_FALSE(e2.move_to_goal.isPending());
	ASSERT_TRUE(e2.move_to_goal.isActive());
	ASSERT_FALSE(e2.move_to_goal.isAborted());
	ASSERT_FALSE(e2.move_to_goal.isSucceeded());
	ASSERT_FALSE(e2.move_to_goal.isEnded());

	// moving to goal...
	for (unsigned int i = 0; i < 25; i++) {
		fsm.process_event(composeEventFsm());
		ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);
	}

	// let's pretend that goal is reached
    task_move_to_goal_ptr->finish();
	auto e3 = composeEventFsm();
	ASSERT_FALSE(e3.move_to_goal.isPending());
    ASSERT_FALSE(e3.move_to_goal.isActive());
	ASSERT_FALSE(e3.move_to_goal.isAborted());
	ASSERT_TRUE(e3.move_to_goal.isSucceeded());
	ASSERT_TRUE(e3.move_to_goal.isEnded());
    fsm.process_event(e3);

	// we end up standing cause goal was reached
    ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	// evaluate if task was properly terminated
	auto e4 = composeEventFsm();
	ASSERT_FALSE(e4.move_to_goal.isPending());
	ASSERT_FALSE(e4.move_to_goal.isActive());
	ASSERT_FALSE(e4.move_to_goal.isAborted());
	ASSERT_FALSE(e4.move_to_goal.isSucceeded());
	ASSERT_FALSE(e4.move_to_goal.isEnded());

	// goal requested
	task_move_to_goal_ptr->request(Pose3(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0));
    auto e5 = composeEventFsm();
	ASSERT_TRUE(e5.move_to_goal.isPending());
	ASSERT_FALSE(e5.move_to_goal.isActive());
	ASSERT_FALSE(e5.move_to_goal.isAborted());
	ASSERT_FALSE(e5.move_to_goal.isSucceeded());
	ASSERT_FALSE(e5.move_to_goal.isEnded());
    fsm.process_event(e5);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	// since state switched, task should be activated with transition handler
	auto e6 = composeEventFsm();
	ASSERT_FALSE(e6.move_to_goal.isPending());
	ASSERT_TRUE(e6.move_to_goal.isActive());
	ASSERT_FALSE(e6.move_to_goal.isAborted());
	ASSERT_FALSE(e6.move_to_goal.isSucceeded());
	ASSERT_FALSE(e6.move_to_goal.isEnded());

	// moving to goal...
	for (unsigned int i = 0; i < 25; i++) {
		fsm.process_event(composeEventFsm());
		ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);
	}

	// goal aborted
	task_move_to_goal_ptr->abort();
	auto e7 = composeEventFsm();
	ASSERT_FALSE(e7.move_to_goal.isPending());
	ASSERT_FALSE(e7.move_to_goal.isActive());
	ASSERT_TRUE(e7.move_to_goal.isAborted());
	ASSERT_FALSE(e7.move_to_goal.isSucceeded());
	ASSERT_FALSE(e7.move_to_goal.isEnded());
    fsm.process_event(e7);

	// after abort, we expect immediate switch to standing state
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	// no need to call finish(), since task was terminated in FSM transition;
	// evaluate if task was properly terminated
	auto e8 = composeEventFsm();
	ASSERT_FALSE(e8.move_to_goal.isPending());
	ASSERT_FALSE(e8.move_to_goal.isActive());
	ASSERT_FALSE(e8.move_to_goal.isAborted());
	ASSERT_FALSE(e8.move_to_goal.isSucceeded());
	ASSERT_FALSE(e8.move_to_goal.isEnded());
}

// Requested another task during first execution
TEST_F(FsmSuperTasksTest, moveToGoalInterrupted) {
	// request some goal
	task_move_to_goal_ptr->request(Pose3(5.0, 5.0, 0.0, 0.0, 0.0, 0.0));
	auto e1 = composeEventFsm();
    ASSERT_TRUE(e1.move_to_goal.isPending());
	ASSERT_FALSE(e1.move_to_goal.isActive());
	ASSERT_FALSE(e1.move_to_goal.isAborted());
	ASSERT_FALSE(e1.move_to_goal.isSucceeded());
	ASSERT_FALSE(e1.move_to_goal.isEnded());
    fsm.process_event(e1);
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);

	// since state switched, task should be activated with transition handler
	auto e2 = composeEventFsm();
	ASSERT_FALSE(e2.move_to_goal.isPending());
	ASSERT_TRUE(e2.move_to_goal.isActive());
	ASSERT_FALSE(e2.move_to_goal.isAborted());
	ASSERT_FALSE(e2.move_to_goal.isSucceeded());
	ASSERT_FALSE(e2.move_to_goal.isEnded());

	// moving to goal...
	for (unsigned int i = 0; i < 25; i++) {
		fsm.process_event(composeEventFsm());
		ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_TO_GOAL);
	}

	// move around task request arrived
	task_move_around_ptr->request();
	auto e3 = composeEventFsm();
	// move to goal is actively executed
	ASSERT_FALSE(e3.move_to_goal.isPending());
	ASSERT_TRUE(e3.move_to_goal.isActive());
	ASSERT_FALSE(e3.move_to_goal.isAborted());
	ASSERT_FALSE(e3.move_to_goal.isSucceeded());
	ASSERT_FALSE(e3.move_to_goal.isEnded());

	// move around is prending (requested, but not started)
	ASSERT_TRUE(e3.move_around.isPending());
	ASSERT_FALSE(e3.move_around.isActive());
	ASSERT_FALSE(e3.move_around.isAborted());
	ASSERT_FALSE(e3.move_around.isSucceeded());
	ASSERT_FALSE(e3.move_around.isEnded());

	fsm.process_event(e3);

	// from move to goal, via stand, to move around (direct switch is not allowed)
	ASSERT_EQ(fsm.current_state(), FsmSuper::State::STAND);

	// moving around...
	for (unsigned int i = 0; i < 25; i++) {
		fsm.process_event(composeEventFsm());
		ASSERT_EQ(fsm.current_state(), FsmSuper::State::MOVE_AROUND);
	}
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
