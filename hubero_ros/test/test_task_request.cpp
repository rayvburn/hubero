#include <gtest/gtest.h>
#include <hubero_ros/task_request_ros.h>

#include <tf2_ros/static_transform_broadcaster.h>

using namespace hubero;

TEST(HuberoTaskRequestRos, taskRequest) {
	auto node = std::make_shared<Node>("task_ros");
    auto task = std::make_shared<TaskBase>(TASK_STAND);
    TaskRequestRos task_request;
    task_request.addTask(TASK_STAND, task);
    task_request.initialize(node, "test", "test_world");

    ASSERT_FALSE(task->isRequested());
    task_request.request(task_request.getTaskName(TASK_STAND));
    ASSERT_TRUE(task->isRequested());
}

/**
 * @brief Evaluates goal transformation to world coordinates
 * @details Requires running `roscore`
 */
TEST(HuberoTaskRequestRos, transformToWorld) {
    const std::string WORLD_FRAME_NAME = "world";
    const std::string GOAL_FRAME_NAME = "map";

    auto node = std::make_shared<Node>("task_ros");
    auto task = std::make_shared<TaskBase>(TASK_STAND);
    TaskRequestRos task_request;
    task_request.initialize(node, "actor", WORLD_FRAME_NAME);

    tf2_ros::StaticTransformBroadcaster tf_static;
    geometry_msgs::TransformStamped tf_world_goal;
    tf_world_goal.header.frame_id = WORLD_FRAME_NAME;
    tf_world_goal.header.stamp = ros::Time::now();
    tf_world_goal.child_frame_id = GOAL_FRAME_NAME;
    tf_world_goal.transform.translation.x = 1.0;
    tf_world_goal.transform.translation.y = 1.0;
    tf_world_goal.transform.translation.z = 0.0;
    // 90 degree rotation
    tf_world_goal.transform.rotation.x = 0.0;
    tf_world_goal.transform.rotation.y = 0.0;
    tf_world_goal.transform.rotation.z = 0.707;
    tf_world_goal.transform.rotation.w = 0.707;
    tf_static.sendTransform(tf_world_goal);

    // without sleep: tf Exception: "world" passed to lookupTransform argument target_frame does not exist.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    Pose3 goal_pose_goal_frame1;
    Pose3 goal_world1 = task_request.transformToWorldFrame(goal_pose_goal_frame1, GOAL_FRAME_NAME);
    ASSERT_NEAR(goal_world1.Pos().X(), 1.0, 1e-06);
    ASSERT_NEAR(goal_world1.Pos().Y(), 1.0, 1e-06);
    ASSERT_NEAR(goal_world1.Pos().Z(), 0.0, 1e-06);
    ASSERT_NEAR(goal_world1.Rot().Roll(), 0.0, 1e-03);
    ASSERT_NEAR(goal_world1.Rot().Pitch(), 0.0, 1e-03);
    ASSERT_NEAR(goal_world1.Rot().Yaw(), IGN_DTOR(90), 1e-03);

    Pose3 goal_pose_goal_frame2(Vector3(2.0, 2.0, 0.0), Quaternion(0.0, 0.0, IGN_DTOR(-90.0)));
    Pose3 goal_world2 = task_request.transformToWorldFrame(goal_pose_goal_frame2, GOAL_FRAME_NAME);
    ASSERT_NEAR(goal_world2.Pos().X(), -1.0, 1e-06);
    ASSERT_NEAR(goal_world2.Pos().Y(), 3.0, 1e-06);
    ASSERT_NEAR(goal_world2.Pos().Z(), 0.0, 1e-06);
    ASSERT_NEAR(goal_world2.Rot().Roll(), 0.0, 1e-03);
    ASSERT_NEAR(goal_world2.Rot().Pitch(), 0.0, 1e-03);
    ASSERT_NEAR(goal_world2.Rot().Yaw(), 0.0, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
