#include <gtest/gtest.h>
#include <hubero_gazebo/localisation_gazebo.h>

using namespace hubero;

TEST(LocalisationActorPlugin, rotationConversion) {
	LocalisationGazebo loc;
	loc.initialize("");

	// TODO: evaluate cases with PITCH > 1.6 rad
	const double ROLL = 3.0 * IGN_PI_4;
	const double PITCH = 0.0;
	const double YAW = -IGN_PI_4;

	Pose3 init_pose(1.0, 2.0, 3.0, ROLL, PITCH, YAW);
	loc.update(init_pose);

	EXPECT_NEAR(init_pose.Rot().Roll(), ROLL, 1e-03);
	EXPECT_NEAR(init_pose.Rot().Pitch(), PITCH, 1e-03);
	EXPECT_NEAR(init_pose.Rot().Yaw(), YAW, 1e-03);

	ASSERT_EQ(loc.getPoseTransformed(), init_pose);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
