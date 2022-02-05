#include <gtest/gtest.h>
#include <hubero_interfaces/navigation_base.h>

#include <iostream>

using namespace hubero;

static const std::string WORLD_FRAME_ID("sim_world");
// same orientation but NavigationBase class ignores orientation anyway
auto POSE_INIT = Pose3(1.5, 1.5, 0.0, 0.0, 0.0, 0.0);
auto POSE_GOAL = Pose3(3.5, -3.5, 0.0, 0.0, 0.0, 0.0);

TEST(HuberoNavigationBase, initialState) {
    NavigationBase nav;
    ASSERT_FALSE(nav.isInitialized());
    nav.initialize("johnny", WORLD_FRAME_ID);
    ASSERT_TRUE(nav.isInitialized());
    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_UNDEFINED);
    ASSERT_FALSE(nav.isPoseAchievable(Pose3(), Pose3(), WORLD_FRAME_ID));
}

TEST(HuberoNavigationBase, normalOperation) {
    NavigationBase nav;
    nav.initialize("johnny", WORLD_FRAME_ID);
    Pose3 pose(POSE_INIT);

    nav.update(pose);
    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_UNDEFINED);

    nav.setGoal(POSE_GOAL, WORLD_FRAME_ID);
    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_PENDING);

    // simulate movement towards goal
    Vector3 to_goal_dir = POSE_GOAL.Pos() - pose.Pos();
    to_goal_dir /= 3.0;
    Pose3 pose_delta(to_goal_dir.X(), to_goal_dir.Y(), to_goal_dir.Z(), 0.0, 0.0, 0.0);

    for (int i = 0; i < 3; i++) {
        pose += pose_delta;
        nav.update(pose);
    }

    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED);

    nav.finish();
    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_TERMINATED);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
