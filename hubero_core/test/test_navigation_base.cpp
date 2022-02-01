#include <gtest/gtest.h>
#include <hubero_interfaces/navigation_base.h>

#include <iostream>

using namespace hubero;

TEST(HuberoNav, initialState) {
    const std::string WORLD_FRAME_ID("sim_world");
    NavigationBase nav;
    ASSERT_FALSE(nav.isInitialized());
    nav.initialize("johnny", WORLD_FRAME_ID);
    ASSERT_TRUE(nav.isInitialized());
    ASSERT_EQ(nav.getFeedback(), TaskFeedbackType::TASK_FEEDBACK_UNDEFINED);
    ASSERT_FALSE(nav.isPoseAchievable(Pose3(), Pose3(), WORLD_FRAME_ID));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
