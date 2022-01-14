#include <gtest/gtest.h>
#include <hubero_ros/task_request_ros.h>

using namespace hubero;

TEST(HuberoTaskRequestRos, taskRequest) {
	auto node = std::make_shared<Node>("task_ros");
    auto task = std::make_shared<TaskBase>(TASK_STAND);
    TaskRequestRos task_request;
    task_request.addTask(TASK_STAND, task);
    task_request.initialize(node, "test");

    ASSERT_FALSE(task->isRequested());
    task_request.request(task_request.getTaskName(TASK_STAND));
    ASSERT_TRUE(task->isRequested());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
