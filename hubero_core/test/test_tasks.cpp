#include <gtest/gtest.h>

#include <hubero_core/tasks/task_follow_object.h>
#include <hubero_core/tasks/task_lie_down.h>
#include <hubero_core/tasks/task_move_around.h>
#include <hubero_core/tasks/task_move_to_goal.h>
#include <hubero_core/tasks/task_run.h>
#include <hubero_core/tasks/task_sit_down.h>
#include <hubero_core/tasks/task_stand.h>
#include <hubero_core/tasks/task_talk.h>
#include <hubero_core/tasks/task_teleop.h>

using namespace hubero;

/// These tests evaluate whether specific task was successfully requested and also check task objective
TEST(HuberoTasks, requestfollowObject) {
    TaskFollowObject task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_FOLLOW_OBJECT);
	ASSERT_FALSE(task.isRequested());
	task.request("robot");
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getFollowedObjectName(), "robot");
}

TEST(HuberoTasks, requestLieDown) {
	TaskLieDown task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_LIE_DOWN);
	ASSERT_FALSE(task.isRequested());
	Vector3 v(1.0, 2.0, 3.0);
	task.request(v, 1.5707);
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getGoalPosition(), v);
	ASSERT_EQ(task.getGoalYaw(), 1.5707);
}

TEST(HuberoTasks, requestMoveAround) {
	TaskMoveAround task(3.0);
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_MOVE_AROUND);
	ASSERT_FALSE(task.isRequested());
	task.request();
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getDistanceGoalReached(), 3.0);
}

TEST(HuberoTasks, requestMoveToGoal) {
	TaskMoveToGoal task(2.5);
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_MOVE_TO_GOAL);
	ASSERT_FALSE(task.isRequested());
	Vector3 v(0.1, 0.2, 0.3);
	Quaternion q(0.7, 0.8, 1.58);
	task.request(Pose3(v, q));
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getDistanceGoalReached(), 2.5);
	ASSERT_EQ(task.getGoal().Pos(), v);
	ASSERT_EQ(task.getGoal().Rot(), q);
}

TEST(HuberoTasks, requestRun) {
	TaskRun task(1.5);
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_RUN);
	ASSERT_FALSE(task.isRequested());
	Vector3 v(0.2, 0.3, 0.4);
	Quaternion q(0.8, 0.9, 1.59);
	task.request(Pose3(v, q));
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getDistanceGoalReached(), 1.5);
	ASSERT_EQ(task.getGoal().Pos(), v);
	ASSERT_EQ(task.getGoal().Rot(), q);
}

TEST(HuberoTasks, requestSitDown) {
	TaskSitDown task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_SIT_DOWN);
	ASSERT_FALSE(task.isRequested());
	task.request(Vector3(0.2, 0.3, 0.4), 0.8);
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getGoalPosition(), Vector3(0.2, 0.3, 0.4));
	ASSERT_EQ(task.getGoalYaw(), 0.8);
}

TEST(HuberoTasks, requestStand) {
	TaskStand task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_STAND);
	ASSERT_FALSE(task.isRequested());
	task.request();
	ASSERT_TRUE(task.isRequested());
}

TEST(HuberoTasks, requestTalk) {
	TaskTalk task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_TALK);
	ASSERT_FALSE(task.isRequested());
	Quaternion q(1.0, 2.0, 3.0);
	Vector3 v(0.25, 0.30, 0.35);
	task.request(Pose3(v, q));
	ASSERT_TRUE(task.isRequested());
	ASSERT_EQ(task.getGoal().Pos(), v);
	ASSERT_EQ(task.getGoal().Rot(), q);
}

TEST(HuberoTasks, requestTeleop) {
	TaskTeleop task;
	ASSERT_EQ(task.getTaskType(), TaskType::TASK_TELEOP);
	ASSERT_FALSE(task.isRequested());
	task.request();
	ASSERT_TRUE(task.isRequested());
	task.setCommand(Vector3(0.25, 0.0, 0.5));
	ASSERT_EQ(task.getCommand().X(), 0.25);
	ASSERT_EQ(task.getCommand().Y(), 0.0);
	ASSERT_EQ(task.getCommand().Z(), 0.5);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

