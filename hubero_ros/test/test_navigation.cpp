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

TEST(HuberoNavRos, idealCovariance) {
    boost::array<double, 36> mat;
    NavigationRos::setIdealCovariance(mat);

    ASSERT_EQ(mat.at(0), 1.0);
    ASSERT_EQ(mat.at(1), 0.0);
    ASSERT_EQ(mat.at(2), 0.0);
    ASSERT_EQ(mat.at(3), 0.0);
    ASSERT_EQ(mat.at(4), 0.0);
    ASSERT_EQ(mat.at(5), 0.0);

    ASSERT_EQ(mat.at(6), 0.0);
    ASSERT_EQ(mat.at(7), 1.0);
    ASSERT_EQ(mat.at(8), 0.0);
    ASSERT_EQ(mat.at(9), 0.0);
    ASSERT_EQ(mat.at(10), 0.0);
    ASSERT_EQ(mat.at(11), 0.0);

    ASSERT_EQ(mat.at(12), 0.0);
    ASSERT_EQ(mat.at(13), 0.0);
    ASSERT_EQ(mat.at(14), 1.0);
    ASSERT_EQ(mat.at(15), 0.0);
    ASSERT_EQ(mat.at(16), 0.0);
    ASSERT_EQ(mat.at(17), 0.0);

    ASSERT_EQ(mat.at(18), 0.0);
    ASSERT_EQ(mat.at(19), 0.0);
    ASSERT_EQ(mat.at(20), 0.0);
    ASSERT_EQ(mat.at(21), 1.0);
    ASSERT_EQ(mat.at(22), 0.0);
    ASSERT_EQ(mat.at(23), 0.0);

    ASSERT_EQ(mat.at(24), 0.0);
    ASSERT_EQ(mat.at(25), 0.0);
    ASSERT_EQ(mat.at(26), 0.0);
    ASSERT_EQ(mat.at(27), 0.0);
    ASSERT_EQ(mat.at(28), 1.0);
    ASSERT_EQ(mat.at(29), 0.0);

    ASSERT_EQ(mat.at(30), 0.0);
    ASSERT_EQ(mat.at(31), 0.0);
    ASSERT_EQ(mat.at(32), 0.0);
    ASSERT_EQ(mat.at(33), 0.0);
    ASSERT_EQ(mat.at(34), 0.0);
    ASSERT_EQ(mat.at(35), 1.0);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
