#include <gtest/gtest.h>
#include <hubero_core/events/task_predicates.h>

#include <iostream>

using namespace hubero;

/**
 * Testing if task predicates, created based on TaskBase, follow intended logic
 */
TEST(HuberoTaskPredicates, predicatesTaskBase) {
    TaskBase task(TaskType::TASK_STAND);
    task.request();
    TaskPredicates pred(task);
    ASSERT_TRUE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    ASSERT_FALSE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    ASSERT_FALSE(pred.isEnded());

    task.activate();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_TRUE(pred.isActive());
    ASSERT_FALSE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    ASSERT_FALSE(pred.isEnded());

    task.abort();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    ASSERT_TRUE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    ASSERT_FALSE(pred.isEnded());

    task.finish();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    // since aborted recently
    ASSERT_TRUE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    ASSERT_TRUE(pred.isEnded());

    task.terminate();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    // terminate 'unaborts'
    ASSERT_FALSE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    // note that terminate erases all flags so 'ended' state will not persist
    ASSERT_FALSE(pred.isEnded());

    // now, finish successfully
    task.request();
    task.activate();
    task.finish();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    ASSERT_FALSE(pred.isAborted());
    ASSERT_TRUE(pred.isSucceeded());
    ASSERT_TRUE(pred.isEnded());

    task.terminate();
    pred = TaskPredicates(task);
    ASSERT_FALSE(pred.isPending());
    ASSERT_FALSE(pred.isActive());
    ASSERT_FALSE(pred.isAborted());
    ASSERT_FALSE(pred.isSucceeded());
    ASSERT_FALSE(pred.isEnded());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
