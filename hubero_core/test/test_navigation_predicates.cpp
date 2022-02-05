#include <gtest/gtest.h>
#include <hubero_core/events/nav_predicates.h>

#include <iostream>

using namespace hubero;

/**
 * Testing NavPredicates based on given TaskFeedbackType
 */
TEST(HuberoNavigationPredicates, feedbackAborted) {
    NavPredicates pred(TASK_FEEDBACK_ABORTED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackActive) {
    NavPredicates pred(TASK_FEEDBACK_ACTIVE);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_TRUE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackEnded) {
    NavPredicates pred(TASK_FEEDBACK_TERMINATED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_TRUE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackLost) {
    NavPredicates pred(TASK_FEEDBACK_LOST);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackPending) {
    NavPredicates pred(TASK_FEEDBACK_PENDING);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackPreempted) {
    NavPredicates pred(TASK_FEEDBACK_PREEMPTED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackPreempting) {
    NavPredicates pred(TASK_FEEDBACK_PREEMPTING);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackRecalled) {
    NavPredicates pred(TASK_FEEDBACK_RECALLED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackRecalling) {
    NavPredicates pred(TASK_FEEDBACK_RECALLING);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_TRUE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackRejected) {
    NavPredicates pred(TASK_FEEDBACK_REJECTED);
    ASSERT_TRUE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackSucceeded) {
    NavPredicates pred(TASK_FEEDBACK_SUCCEEDED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_TRUE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

TEST(HuberoNavigationPredicates, feedbackUndefined) {
    NavPredicates pred(TASK_FEEDBACK_UNDEFINED);
    ASSERT_FALSE(pred.isNavigationGoalRejected());
    ASSERT_FALSE(pred.isNavigationActive());
    ASSERT_FALSE(pred.isNavigationGoalCancelled());
    ASSERT_FALSE(pred.isNavigationSucceeded());
    ASSERT_FALSE(pred.isNavigationEnded());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
