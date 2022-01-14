#include <hubero_ros/task_request_ros.h>

namespace hubero {

const std::string TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX = "_name";
const std::chrono::milliseconds TaskRequestRos::TASK_FEEDBACK_PERIOD = std::chrono::milliseconds(500);

TaskRequestRos::TaskRequestRos(): TaskRequestBase::TaskRequestBase() {}

void TaskRequestRos::initialize(std::shared_ptr<Node> node_ptr, const std::string& actor_name) {
    if (node_ptr == nullptr) {
        HUBERO_LOG("[TaskRequestRos] Pointer to Node is null, action servers will not be started\r\n");
        return;
    }

    node_ptr_ = node_ptr;

    // convert task IDs to task names
    auto name_task_stand = TaskRequestBase::getTaskName(TASK_STAND);
	auto name_task_move_to_goal = TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL);
	auto name_task_move_around = TaskRequestBase::getTaskName(TASK_MOVE_AROUND);
	auto name_task_lie_down = TaskRequestBase::getTaskName(TASK_LIE_DOWN);
	auto name_task_sit_down = TaskRequestBase::getTaskName(TASK_SIT_DOWN);
	auto name_task_follow_object = TaskRequestBase::getTaskName(TASK_FOLLOW_OBJECT);
	auto name_task_teleop = TaskRequestBase::getTaskName(TASK_TELEOP);
	auto name_task_run = TaskRequestBase::getTaskName(TASK_RUN);
	auto name_task_talk = TaskRequestBase::getTaskName(TASK_TALK);

    // create action servers
    as_follow_object_ = std::make_shared<ActionServer<hubero_ros_msgs::FollowObjectAction>>(
        *node_ptr_->getNodeHandlePtr(),
        actor_name + "/" + name_task_follow_object,
        std::bind(&TaskRequestRos::actionCbFollowObject, this, std::placeholders::_1),
        false
    );
    as_follow_object_->start();

    as_lie_down_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownAction>>(
        *node_ptr_->getNodeHandlePtr(),
        actor_name + "/" + name_task_lie_down,
        std::bind(&TaskRequestRos::actionCbLieDown, this, std::placeholders::_1),
        false
    );
    as_lie_down_->start();

    as_lie_down_object_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownObjectAction>>(
        *node_ptr_->getNodeHandlePtr(),
        actor_name + "/" + name_task_lie_down + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
        std::bind(&TaskRequestRos::actionCbLieDownObject, this, std::placeholders::_1),
        false
    );
    as_lie_down_object_->start();

    as_move_to_goal_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToGoalAction>>(
        *node_ptr_->getNodeHandlePtr(),
        actor_name + "/" + name_task_move_to_goal,
        std::bind(&TaskRequestRos::actionCbMoveToGoal, this, std::placeholders::_1),
        false
    );
    as_move_to_goal_->start();

    as_move_to_object_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToObjectAction>>(
        *node_ptr_->getNodeHandlePtr(),
        actor_name + "/" + name_task_move_to_goal + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
        std::bind(&TaskRequestRos::actionCbMoveToObject, this, std::placeholders::_1),
        false
    );
    as_move_to_object_->start();
}

void TaskRequestRos::actionCbFollowObject(const hubero_ros_msgs::FollowObjectGoalConstPtr& goal) {
    bool request_processed_ok = request(TASK_FOLLOW_OBJECT, goal->object_name);
    actionCbHandler<hubero_ros_msgs::FollowObjectResult, hubero_ros_msgs::FollowObjectFeedback>(
        request_processed_ok,
        TASK_FOLLOW_OBJECT,
        as_follow_object_
    );
}

void TaskRequestRos::actionCbMoveToGoal(const hubero_ros_msgs::MoveToGoalGoalConstPtr& goal) {
    bool request_processed_ok = false; // request(TASK_MOVE_TO_GOAL, goal->...);
    actionCbHandler<hubero_ros_msgs::MoveToGoalResult, hubero_ros_msgs::MoveToGoalFeedback>(
        request_processed_ok,
        TASK_MOVE_TO_GOAL,
        as_move_to_goal_
    );
}

void TaskRequestRos::actionCbMoveToObject(const hubero_ros_msgs::MoveToObjectGoalConstPtr& goal) {
    bool request_processed_ok = false; // request(TASK_MOVE_TO_GOAL, goal->...);
    actionCbHandler<hubero_ros_msgs::MoveToObjectResult, hubero_ros_msgs::MoveToObjectFeedback>(
        request_processed_ok,
        TASK_MOVE_TO_GOAL,
        as_move_to_object_
    );
}

void TaskRequestRos::actionCbLieDown(const hubero_ros_msgs::LieDownGoalConstPtr& goal) {
    bool request_processed_ok = false; // request(TASK_LIE_DOWN, goal->...);
    actionCbHandler<hubero_ros_msgs::LieDownResult, hubero_ros_msgs::LieDownFeedback>(
        request_processed_ok,
        TASK_LIE_DOWN,
        as_lie_down_
    );
}

void TaskRequestRos::actionCbLieDownObject(const hubero_ros_msgs::LieDownObjectGoalConstPtr& goal) {
    bool request_processed_ok = false; // request(TASK_LIE_DOWN, goal->...);
    actionCbHandler<hubero_ros_msgs::LieDownObjectResult, hubero_ros_msgs::LieDownObjectFeedback>(
        request_processed_ok,
        TASK_LIE_DOWN,
        as_lie_down_object_
    );
}

} // namespace hubero
