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

	// prepare task namespace
	std::string task_ns = actor_name + "/" + node_ptr->getTaskNamespaceName() + "/";

	// create action servers
	as_follow_object_ = std::make_shared<ActionServer<hubero_ros_msgs::FollowObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_follow_object,
		std::bind(&TaskRequestRos::actionCbFollowObject, this, std::placeholders::_1),
		false
	);
	as_follow_object_->start();

	as_lie_down_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_lie_down,
		std::bind(&TaskRequestRos::actionCbLieDown, this, std::placeholders::_1),
		false
	);
	as_lie_down_->start();

	as_lie_down_object_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_lie_down + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbLieDownObject, this, std::placeholders::_1),
		false
	);
	as_lie_down_object_->start();

	as_move_around_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveAroundAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_around,
		std::bind(&TaskRequestRos::actionCbMoveAround, this, std::placeholders::_1),
		false
	);
	as_move_around_->start();

	as_move_to_goal_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToGoalAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_to_goal,
		std::bind(&TaskRequestRos::actionCbMoveToGoal, this, std::placeholders::_1),
		false
	);
	as_move_to_goal_->start();

	as_move_to_object_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_to_goal + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbMoveToObject, this, std::placeholders::_1),
		false
	);
	as_move_to_object_->start();

	as_run_ = std::make_shared<ActionServer<hubero_ros_msgs::RunAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_run,
		std::bind(&TaskRequestRos::actionCbRun, this, std::placeholders::_1),
		false
	);
	as_run_->start();

	as_sit_down_ = std::make_shared<ActionServer<hubero_ros_msgs::SitDownAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_sit_down,
		std::bind(&TaskRequestRos::actionCbSitDown, this, std::placeholders::_1),
		false
	);
	as_sit_down_->start();

	as_sit_down_object_ = std::make_shared<ActionServer<hubero_ros_msgs::SitDownObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_sit_down + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbSitDownObject, this, std::placeholders::_1),
		false
	);
	as_sit_down_object_->start();

	as_stand_ = std::make_shared<ActionServer<hubero_ros_msgs::StandAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_stand,
		std::bind(&TaskRequestRos::actionCbStand, this, std::placeholders::_1),
		false
	);
	as_stand_->start();

	as_talk_ = std::make_shared<ActionServer<hubero_ros_msgs::TalkAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_talk,
		std::bind(&TaskRequestRos::actionCbTalk, this, std::placeholders::_1),
		false
	);
	as_talk_->start();

	as_talk_object_ = std::make_shared<ActionServer<hubero_ros_msgs::TalkObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_talk + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbTalkObject, this, std::placeholders::_1),
		false
	);
	as_talk_object_->start();

	as_teleop_ = std::make_shared<ActionServer<hubero_ros_msgs::TeleopAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_teleop,
		std::bind(&TaskRequestRos::actionCbTeleop, this, std::placeholders::_1),
		false
	);
	as_teleop_->start();
}

void TaskRequestRos::actionCbFollowObject(const hubero_ros_msgs::FollowObjectGoalConstPtr& goal) {
	bool request_processed_ok = request(TASK_FOLLOW_OBJECT, goal->object_name);
	actionCbHandler<hubero_ros_msgs::FollowObjectResult, hubero_ros_msgs::FollowObjectFeedback>(
		request_processed_ok,
		TASK_FOLLOW_OBJECT,
		as_follow_object_
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

void TaskRequestRos::actionCbMoveAround(const hubero_ros_msgs::MoveAroundGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_MOVE_AROUND, goal->...);
	actionCbHandler<hubero_ros_msgs::MoveAroundResult, hubero_ros_msgs::MoveAroundFeedback>(
		request_processed_ok,
		TASK_MOVE_AROUND,
		as_move_around_
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

void TaskRequestRos::actionCbRun(const hubero_ros_msgs::RunGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_RUN, goal->...);
	actionCbHandler<hubero_ros_msgs::RunResult, hubero_ros_msgs::RunFeedback>(
		request_processed_ok,
		TASK_RUN,
		as_run_
	);
}

void TaskRequestRos::actionCbSitDown(const hubero_ros_msgs::SitDownGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_SIT_DOWN, goal->...);
	actionCbHandler<hubero_ros_msgs::SitDownResult, hubero_ros_msgs::SitDownFeedback>(
		request_processed_ok,
		TASK_SIT_DOWN,
		as_sit_down_
	);
}

void TaskRequestRos::actionCbSitDownObject(const hubero_ros_msgs::SitDownObjectGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_SIT_DOWN, goal->...);
	actionCbHandler<hubero_ros_msgs::SitDownObjectResult, hubero_ros_msgs::SitDownObjectFeedback>(
		request_processed_ok,
		TASK_SIT_DOWN,
		as_sit_down_object_
	);
}

void TaskRequestRos::actionCbStand(const hubero_ros_msgs::StandGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_STAND, goal->...);
	actionCbHandler<hubero_ros_msgs::StandResult, hubero_ros_msgs::StandFeedback>(
		request_processed_ok,
		TASK_STAND,
		as_stand_
	);
}

void TaskRequestRos::actionCbTalk(const hubero_ros_msgs::TalkGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_TALK, goal->...);
	actionCbHandler<hubero_ros_msgs::TalkResult, hubero_ros_msgs::TalkFeedback>(
		request_processed_ok,
		TASK_TALK,
		as_talk_
	);
}

void TaskRequestRos::actionCbTalkObject(const hubero_ros_msgs::TalkObjectGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_TALK, goal->...);
	actionCbHandler<hubero_ros_msgs::TalkObjectResult, hubero_ros_msgs::TalkObjectFeedback>(
		request_processed_ok,
		TASK_TALK,
		as_talk_object_
	);
}

void TaskRequestRos::actionCbTeleop(const hubero_ros_msgs::TeleopGoalConstPtr& goal) {
	bool request_processed_ok = false; // request(TASK_TELEOP, goal->...);
	actionCbHandler<hubero_ros_msgs::TeleopResult, hubero_ros_msgs::TeleopFeedback>(
		request_processed_ok,
		TASK_TELEOP,
		as_teleop_
	);
}

} // namespace hubero
