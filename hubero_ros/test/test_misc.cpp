#include <gtest/gtest.h>
#include <hubero_ros/utils/misc.h>

using namespace hubero;

TEST(HuberoRosMisc, idealCovariance) {
    boost::array<double, 36> mat;
    setIdealCovariance(mat);

    ASSERT_NEAR(mat.at(0), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(1), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(2), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(3), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(4), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(5), 0.0, 1e-03);

    ASSERT_NEAR(mat.at(6), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(7), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(8), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(9), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(10), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(11), 0.0, 1e-03);

    ASSERT_NEAR(mat.at(12), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(13), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(14), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(15), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(16), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(17), 0.0, 1e-03);

    ASSERT_NEAR(mat.at(18), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(19), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(20), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(21), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(22), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(23), 0.0, 1e-03);

    ASSERT_NEAR(mat.at(24), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(25), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(26), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(27), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(28), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(29), 0.0, 1e-03);

    ASSERT_NEAR(mat.at(30), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(31), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(32), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(33), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(34), 0.0, 1e-03);
    ASSERT_NEAR(mat.at(35), 0.0, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
