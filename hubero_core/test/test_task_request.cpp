#include <gtest/gtest.h>

#include <hubero_interfaces/task_request_base.h>

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

/**
 * @brief try to request tasks via TaskRequest interface class
 * @details prepares TaskRequestBase so all available tasks are added to its internal map
 */
class TaskRequestTest: public ::testing::Test {
protected:
	TaskRequestTest():
		tfo_ptr(std::make_shared<TaskFollowObject>()),
		tld_ptr(std::make_shared<TaskLieDown>()),
		tma_ptr(std::make_shared<TaskMoveAround>()),
		tmtg_ptr(std::make_shared<TaskMoveToGoal>()),
		tr_ptr(std::make_shared<TaskRun>()),
		tsd_ptr(std::make_shared<TaskSitDown>()),
		ts_ptr(std::make_shared<TaskStand>()),
		tt_ptr(std::make_shared<TaskTalk>()),
		ttel_ptr(std::make_shared<TaskTeleop>()) {}

	void SetUp() override {
		// TODO: add task may take task ptr only
		task_req.addTask(tfo_ptr->getTaskType(), tfo_ptr);
		task_req.addTask(tld_ptr->getTaskType(), tld_ptr);
		task_req.addTask(tma_ptr->getTaskType(), tma_ptr);
		task_req.addTask(tmtg_ptr->getTaskType(), tmtg_ptr);
		task_req.addTask(tr_ptr->getTaskType(), tr_ptr);
		task_req.addTask(tsd_ptr->getTaskType(), tsd_ptr);
		task_req.addTask(ts_ptr->getTaskType(), ts_ptr);
		task_req.addTask(tt_ptr->getTaskType(), tt_ptr);
		task_req.addTask(ttel_ptr->getTaskType(), ttel_ptr);
	}

	// void TearDown() override {}

	std::shared_ptr<TaskFollowObject> tfo_ptr;
	std::shared_ptr<TaskLieDown> tld_ptr;
	std::shared_ptr<TaskMoveAround> tma_ptr;
	std::shared_ptr<TaskMoveToGoal> tmtg_ptr;
	std::shared_ptr<TaskRun> tr_ptr;
	std::shared_ptr<TaskSitDown> tsd_ptr;
	std::shared_ptr<TaskStand> ts_ptr;
	std::shared_ptr<TaskTalk> tt_ptr;
	std::shared_ptr<TaskTeleop> ttel_ptr;
	TaskRequestBase task_req;
};

TEST_F(TaskRequestTest, trFollowObject) {
	// follow object
	ASSERT_FALSE(tfo_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(tfo_ptr->getTaskType()));
	ASSERT_FALSE(tfo_ptr->isRequested());
	// valid request
	task_req.request(tfo_ptr->getTaskType(), std::string("robot_to_follow"));
	ASSERT_TRUE(tfo_ptr->isRequested());
}

TEST_F(TaskRequestTest, trLieDown) {
	// lie down
	ASSERT_FALSE(tld_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(tld_ptr->getTaskType()));
	ASSERT_FALSE(tld_ptr->isRequested());
	ASSERT_FALSE(task_req.request(tld_ptr->getTaskType(), Pose3()));
	ASSERT_FALSE(tld_ptr->isRequested());
	// valid request
	task_req.request(tld_ptr->getTaskType(), Vector3(), double(0.0));
	ASSERT_TRUE(tld_ptr->isRequested());
}

TEST_F(TaskRequestTest, trMoveAround) {
	// move around
	ASSERT_FALSE(tma_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(tma_ptr->getTaskType(), double(0.0)));
	ASSERT_FALSE(tma_ptr->isRequested());
	// valid request
	task_req.request(tma_ptr->getTaskType());
	ASSERT_TRUE(tma_ptr->isRequested());
}

TEST_F(TaskRequestTest, trMoveToGoal) {
	// move to goal
	ASSERT_FALSE(tmtg_ptr->isRequested());
	// wrong request
	// FIXME: passing to request() 1 arg, e.g., Vector3(), fails the test
	ASSERT_FALSE(task_req.request(tmtg_ptr->getTaskType()));
	ASSERT_FALSE(tmtg_ptr->isRequested());
	// valid request
	task_req.request(tmtg_ptr->getTaskType(), Pose3());
	ASSERT_TRUE(tmtg_ptr->isRequested());
}

TEST_F(TaskRequestTest, trRun) {
	// run
	ASSERT_FALSE(tr_ptr->isRequested());
	// wrong request
	// FIXME: passing to request() 1 arg, e.g., Vector3(), fails the test
	ASSERT_FALSE(task_req.request(tr_ptr->getTaskType()));
	ASSERT_FALSE(tr_ptr->isRequested());
	// valid request
	task_req.request(tr_ptr->getTaskType(), Pose3());
	ASSERT_TRUE(tr_ptr->isRequested());
}

TEST_F(TaskRequestTest, trSitDown) {
	// sit down
	ASSERT_FALSE(tsd_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(tsd_ptr->getTaskType()));
	ASSERT_FALSE(tsd_ptr->isRequested());
	// valid request
	task_req.request(tsd_ptr->getTaskType(), Vector3(), double(0.0));
	ASSERT_TRUE(tsd_ptr->isRequested());
}

TEST_F(TaskRequestTest, trStand) {
	// stand
	ASSERT_FALSE(ts_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(ts_ptr->getTaskType(), Pose3()));
	ASSERT_FALSE(ts_ptr->isRequested());
	// valid request
	task_req.request(ts_ptr->getTaskType());
	ASSERT_TRUE(ts_ptr->isRequested());
}

TEST_F(TaskRequestTest, trTalk) {
	// talk
	ASSERT_FALSE(tt_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(tt_ptr->getTaskType()));
	ASSERT_FALSE(tt_ptr->isRequested());
	// valid request
	task_req.request(tt_ptr->getTaskType(), Pose3());
	ASSERT_TRUE(tt_ptr->isRequested());
}

TEST_F(TaskRequestTest, trTeleop) {
	// teleop
	ASSERT_FALSE(ttel_ptr->isRequested());
	// wrong request
	ASSERT_FALSE(task_req.request(ttel_ptr->getTaskType(), Pose3()));
	ASSERT_FALSE(ttel_ptr->isRequested());
	// valid request
	task_req.request(ttel_ptr->getTaskType());
	ASSERT_TRUE(ttel_ptr->isRequested());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
