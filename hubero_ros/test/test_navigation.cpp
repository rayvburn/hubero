#include <gtest/gtest.h>
#include <hubero_ros/navigation_ros.h>

using namespace hubero;

TEST(HuberoNavRos, init) {
	auto node = std::make_shared<Node>("nav_ros");
    NavigationRos nav;
    nav.initialize(node, "actor", "sim_world", Pose3());
    ASSERT_TRUE(nav.isInitialized());
}

TEST(HuberoNavRos, setGoal) {
	auto node = std::make_shared<Node>("nav_ros");
    NavigationRos nav;
    nav.initialize(node, "actor", "sim_world", Pose3());
    nav.setGoal(Pose3(Vector3(1.0, 2.0, 3.0), Quaternion()), "simulated_world");
    ASSERT_EQ(nav.getGoalPose().Pos().X(), 1.0);
    ASSERT_EQ(nav.getGoalPose().Pos().Y(), 2.0);
    ASSERT_EQ(nav.getGoalPose().Pos().Z(), 3.0);
    ASSERT_EQ(nav.getGoalFrame(), "simulated_world");
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
