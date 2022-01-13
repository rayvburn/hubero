#include <gtest/gtest.h>
#include <hubero_core/tasks/task_move_to_goal.h>

using namespace hubero;

TEST(HuberoTaskBase, idle) {
	TaskBase task(TASK_STAND);
	ASSERT_FALSE(task.isRequested());
	ASSERT_FALSE(task.isAborted());
	ASSERT_FALSE(task.isActive());
	ASSERT_FALSE(task.isFinished());
	ASSERT_EQ(task.getTaskFeedbackType(), TASK_FEEDBACK_UNDEFINED);
}

TEST(HuberoTaskBase, request) {
	TaskBase task(TASK_STAND);
	task.request();
	ASSERT_TRUE(task.isRequested());
	ASSERT_FALSE(task.isAborted());
	ASSERT_FALSE(task.isActive());
	ASSERT_FALSE(task.isFinished());
	ASSERT_EQ(task.getTaskFeedbackType(), TASK_FEEDBACK_PENDING);
}

TEST(HuberoTaskBase, abort) {
	TaskBase task(TASK_STAND);
	task.abort();
	ASSERT_FALSE(task.isRequested());
	ASSERT_TRUE(task.isAborted());
	ASSERT_FALSE(task.isActive());
	ASSERT_FALSE(task.isFinished());
	ASSERT_EQ(task.getTaskFeedbackType(), TASK_FEEDBACK_ABORTED);
}

TEST(HuberoTaskBase, activate) {
	TaskBase task(TASK_STAND);
	task.activate();
	ASSERT_FALSE(task.isRequested());
	ASSERT_FALSE(task.isAborted());
	ASSERT_TRUE(task.isActive());
	ASSERT_FALSE(task.isFinished());
	ASSERT_EQ(task.getTaskFeedbackType(), TASK_FEEDBACK_ACTIVE);
}

TEST(HuberoTaskBase, terminate) {
	TaskBase task(TASK_STAND);
	task.terminate();
	ASSERT_FALSE(task.isRequested());
	ASSERT_FALSE(task.isAborted());
	ASSERT_FALSE(task.isActive());
	ASSERT_FALSE(task.isFinished());
	ASSERT_EQ(task.getTaskFeedbackType(), TASK_FEEDBACK_SUCCEEDED);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
