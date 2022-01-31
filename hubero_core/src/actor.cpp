#include <hubero_core/actor.h>
#include <hubero_common/logger.h>

namespace hubero {

Actor::Actor():
	actor_sim_name_("unnamed"),
	task_stand_ptr_(std::make_shared<TaskStand>()),
	task_move_to_goal_ptr_(std::make_shared<TaskMoveToGoal>()),
	task_move_around_ptr_(std::make_shared<TaskMoveAround>()),
	task_lie_down_ptr_(std::make_shared<TaskLieDown>()),
	task_sit_down_ptr_(std::make_shared<TaskSitDown>()),
	task_follow_object_ptr_(std::make_shared<TaskFollowObject>()),
	task_teleop_ptr_(std::make_shared<TaskTeleop>()),
	task_run_ptr_(std::make_shared<TaskRun>()),
	task_talk_ptr_(std::make_shared<TaskTalk>())
{
	TaskBase::addBasicBehaviourHandler(BB_STAND, std::bind(&Actor::bbStand, this));
	TaskBase::addBasicBehaviourHandler(BB_ALIGN_TO_TARGET, std::bind(&Actor::bbAlignToTarget, this));
	TaskBase::addBasicBehaviourHandler(BB_MOVE_TO_GOAL, std::bind(&Actor::bbMoveToGoal, this));
	TaskBase::addBasicBehaviourHandler(BB_CHOOSE_NEW_GOAL, std::bind(&Actor::bbChooseNewGoal, this));
	TaskBase::addBasicBehaviourHandler(BB_AWAIT_OBJECT_MOVEMENT, std::bind(&Actor::bbAwaitObjectMovement, this));
	TaskBase::addBasicBehaviourHandler(BB_LIE_DOWN, std::bind(&Actor::bbLieDown, this));
	TaskBase::addBasicBehaviourHandler(BB_STAND_UP_FROM_LYING, std::bind(&Actor::bbStandUpFromLying, this));
	TaskBase::addBasicBehaviourHandler(BB_SIT_DOWN, std::bind(&Actor::bbSitDown, this));
	TaskBase::addBasicBehaviourHandler(BB_STAND_UP_FROM_SITTING, std::bind(&Actor::bbStandUpFromSitting, this));
	TaskBase::addBasicBehaviourHandler(BB_RUN, std::bind(&Actor::bbRun, this));
	TaskBase::addBasicBehaviourHandler(BB_TALK, std::bind(&Actor::bbTalk, this));
	TaskBase::addBasicBehaviourHandler(BB_TELEOP, std::bind(&Actor::bbTeleop, this));

	state_memory_updater_map_ = decltype(state_memory_updater_map_){
		{FsmSuper::State::STAND, std::bind(&Actor::updateMemoryFromTaskStand, this, std::placeholders::_1)},
		{FsmSuper::State::MOVE_TO_GOAL, std::bind(&Actor::updateMemoryFromTaskMoveToGoal, this, std::placeholders::_1)},
		{FsmSuper::State::MOVE_AROUND, std::bind(&Actor::updateMemoryFromTaskMoveAround, this, std::placeholders::_1)},
		{FsmSuper::State::LIE_DOWN, std::bind(&Actor::updateMemoryFromTaskLieDown, this, std::placeholders::_1)},
		{FsmSuper::State::SIT_DOWN, std::bind(&Actor::updateMemoryFromTaskSitDown, this, std::placeholders::_1)},
		{FsmSuper::State::FOLLOW_OBJECT, std::bind(&Actor::updateMemoryFromTaskFollowObject, this, std::placeholders::_1)},
		{FsmSuper::State::TELEOP, std::bind(&Actor::updateMemoryFromTaskTeleop, this, std::placeholders::_1)},
		{FsmSuper::State::RUN, std::bind(&Actor::updateMemoryFromTaskRun, this, std::placeholders::_1)},
		{FsmSuper::State::TALK, std::bind(&Actor::updateMemoryFromTaskTalk, this, std::placeholders::_1)}
	};

	state_tf_exec_map_ = decltype(state_tf_exec_map_){
		{FsmSuper::State::STAND, std::bind(&Actor::executeTaskStand, this)},
		{FsmSuper::State::MOVE_TO_GOAL, std::bind(&Actor::executeTaskMoveToGoal, this)},
		{FsmSuper::State::MOVE_AROUND, std::bind(&Actor::executeTaskMoveAround, this)},
		{FsmSuper::State::LIE_DOWN, std::bind(&Actor::executeTaskLieDown, this)},
		{FsmSuper::State::SIT_DOWN, std::bind(&Actor::executeTaskSitDown, this)},
		{FsmSuper::State::FOLLOW_OBJECT, std::bind(&Actor::executeTaskFollowObject, this)},
		{FsmSuper::State::TELEOP, std::bind(&Actor::executeTaskTeleop, this)},
		{FsmSuper::State::RUN, std::bind(&Actor::executeTaskRun, this)},
		{FsmSuper::State::TALK, std::bind(&Actor::executeTaskTalk, this)}
	};
}

void Actor::initializeSim(
	const std::string& actor_sim_name,
	std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr,
	std::shared_ptr<hubero::ModelControlBase> model_control_ptr,
	std::shared_ptr<hubero::LocalisationBase> localisation_ptr
) {
	actor_sim_name_ = actor_sim_name;
	animation_control_ptr_ = animation_control_ptr;
	model_control_ptr_ = model_control_ptr;
	localisation_ptr_  = localisation_ptr;

	// since name was set, update FSM logger preamble too
	fsm_.setLoggerPreamble(actor_sim_name_);
}

void Actor::initializeNav(std::shared_ptr<hubero::NavigationBase> navigation_ptr) {
	navigation_ptr_ = navigation_ptr;
}

void Actor::initializeTask(std::shared_ptr<hubero::TaskRequestBase> task_request_ptr) {
	task_request_ptr_ = task_request_ptr;
	task_request_ptr_->addTask(task_stand_ptr_->getTaskType(), task_stand_ptr_);
	task_request_ptr_->addTask(task_move_to_goal_ptr_->getTaskType(), task_move_to_goal_ptr_);
	task_request_ptr_->addTask(task_move_around_ptr_->getTaskType(), task_move_around_ptr_);
	task_request_ptr_->addTask(task_lie_down_ptr_->getTaskType(), task_lie_down_ptr_);
	task_request_ptr_->addTask(task_sit_down_ptr_->getTaskType(), task_sit_down_ptr_);
	task_request_ptr_->addTask(task_follow_object_ptr_->getTaskType(), task_follow_object_ptr_);
	task_request_ptr_->addTask(task_teleop_ptr_->getTaskType(), task_teleop_ptr_);
	task_request_ptr_->addTask(task_run_ptr_->getTaskType(), task_run_ptr_);
	task_request_ptr_->addTask(task_talk_ptr_->getTaskType(), task_talk_ptr_);

	task_map_ = decltype(task_map_){
		{TaskType::TASK_FOLLOW_OBJECT, task_follow_object_ptr_},
		{TaskType::TASK_LIE_DOWN, task_lie_down_ptr_},
		{TaskType::TASK_MOVE_AROUND, task_move_around_ptr_},
		{TaskType::TASK_MOVE_TO_GOAL, task_move_to_goal_ptr_},
		{TaskType::TASK_RUN, task_run_ptr_},
		{TaskType::TASK_SIT_DOWN, task_sit_down_ptr_},
		{TaskType::TASK_STAND, task_stand_ptr_},
		{TaskType::TASK_TALK, task_talk_ptr_},
		{TaskType::TASK_TELEOP, task_teleop_ptr_}
	};
}

void Actor::update(const Time& time) {
	if (!isInitialized()) {
		HUBERO_LOG("Actor class not initialized yet!\r\n");
		return;
	}

	// input data
	mem_.setTime(time);
	mem_.setPose(localisation_ptr_->getPose());

	// update internal memory, based on internal buffer content that is specific to a certain task
	auto mem_update_it = state_memory_updater_map_.find(static_cast<FsmSuper::State>(fsm_.current_state()));
	if (mem_update_it != state_memory_updater_map_.end()) {
		mem_update_it->second(mem_);
	} else {
		HUBERO_LOG("FsmSuper has a state that Actor class is not aware of! Cannot update internal memory buffer\r\n");
	}

	// execute transition function of the specific task and update FSM predicates
	auto tf_exec_it = state_tf_exec_map_.find(static_cast<FsmSuper::State>(fsm_.current_state()));
	if (tf_exec_it != state_tf_exec_map_.end()) {
		tf_exec_it->second();
	} else {
		HUBERO_LOG("FsmSuper has a state that Actor class is not aware of! Cannot execute transition function\r\n");
	}

	// output data
	localisation_ptr_->update(mem_.getPoseCurrent());

	navigation_ptr_->update(
		localisation_ptr_->getPose(),
		localisation_ptr_->getVelocityLinear(),
		localisation_ptr_->getVelocityAngular()
	);

	model_control_ptr_->update(
		localisation_ptr_->getPoseSimulator(),
		localisation_ptr_->getVelocityAngular(),
		localisation_ptr_->getVelocityLinear(),
		localisation_ptr_->getAccelerationAngular(),
		localisation_ptr_->getAccelerationLinear()
	);

	// Task-related FSMs got updated at the end of transition function execution so now update highest level FSM
	updateFsmSuper();
}

bool Actor::isInitialized() const {
	return animation_control_ptr_ != nullptr
		&& model_control_ptr_ != nullptr
		&& localisation_ptr_ != nullptr
		&& navigation_ptr_ != nullptr
		&& task_request_ptr_ != nullptr;
}

// static
Vector3 Actor::computeCommandToGlobalCs(const double& yaw_actor, const Vector3& cmd_vel_local) {
	// slide 38 at https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture3-RobotMotion.pdf
	ignition::math::Matrix3d r(
		cos(yaw_actor), 0.0, 0.0,
		sin(yaw_actor), 0.0, 0.0,
		0.0, 0.0, 1.0
	);
	return r * cmd_vel_local;
}

void Actor::updateMemoryFromTaskStand(InternalMemory& m) {
	m.setBasicBehaviour(task_stand_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskMoveToGoal(InternalMemory& m) {
	m.setBasicBehaviour(task_move_to_goal_ptr_->getBasicBehaviour());
	m.setGoal(task_move_to_goal_ptr_->getGoal());
}

void Actor::updateMemoryFromTaskMoveAround(InternalMemory& m) {
	m.setBasicBehaviour(task_move_around_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskLieDown(InternalMemory& m) {
	m.setBasicBehaviour(task_lie_down_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskSitDown(InternalMemory& m) {
	m.setBasicBehaviour(task_sit_down_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskFollowObject(InternalMemory& m) {
	m.setBasicBehaviour(task_follow_object_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskTeleop(InternalMemory& m) {
	m.setBasicBehaviour(task_teleop_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskRun(InternalMemory& m) {
	m.setBasicBehaviour(task_run_ptr_->getBasicBehaviour());
}

void Actor::updateMemoryFromTaskTalk(InternalMemory& m) {
	m.setBasicBehaviour(task_talk_ptr_->getBasicBehaviour());
}

void Actor::executeTaskStand() {
	auto event = prepareTaskFsmUpdate<EventFsmBasic>(task_stand_ptr_);
	task_stand_ptr_->execute(event);
}

void Actor::executeTaskMoveToGoal() {
	auto event = prepareTaskFsmUpdate<EventFsmBasic>(task_move_to_goal_ptr_);
	task_move_to_goal_ptr_->execute(event);
}

void Actor::executeTaskMoveAround() {
	auto event = prepareTaskFsmUpdate<EventFsmMoveAround>(task_move_around_ptr_);
	event.goal_reached = mem_.getDistanceToGoal() <= task_move_around_ptr_->getDistanceGoalReached();
	event.goal_selected = mem_.getDistanceToGoal() > task_move_around_ptr_->getDistanceGoalReached();
	task_move_around_ptr_->execute(event);
}

void Actor::executeTaskLieDown() {
	auto event = prepareTaskFsmUpdate<EventFsmLieDown>(task_lie_down_ptr_);
	event.goal_reached = navigation_ptr_->getFeedback() == TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
	event.lied_down = animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_LIE_DOWN;
	event.stood_up = animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_STAND;
	task_lie_down_ptr_->execute(event);
}

void Actor::executeTaskSitDown() {
	auto event = prepareTaskFsmUpdate<EventFsmSitDown>(task_sit_down_ptr_);
	event.goal_reached = navigation_ptr_->getFeedback() == TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
	event.sat_down = animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_SITTING;
	event.stood_up = animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_STAND;
	task_sit_down_ptr_->execute(event);
}

void Actor::executeTaskFollowObject() {
	auto event = prepareTaskFsmUpdate<EventFsmFollowObject>(task_follow_object_ptr_);
	event.object_nearby = mem_.getDistanceToGoal() <= task_follow_object_ptr_->getDistanceNearby();
	task_follow_object_ptr_->execute(event);
}

void Actor::executeTaskTeleop() {
	auto event = prepareTaskFsmUpdate<EventFsmBasic>(task_teleop_ptr_);
	task_teleop_ptr_->execute(event);
}

void Actor::executeTaskRun() {
	auto event = prepareTaskFsmUpdate<EventFsmBasic>(task_run_ptr_);
	task_run_ptr_->execute(event);
}

void Actor::executeTaskTalk() {
	auto event = prepareTaskFsmUpdate<EventFsmTalk>(task_talk_ptr_);
	event.goal_reached = navigation_ptr_->getFeedback() == TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
	task_talk_ptr_->execute(event);
}

void Actor::terminateOtherTasks(TaskType task_type_current) {
	for (const auto& task: task_map_) {
		if (task.first != task_type_current) {
			task.second->terminate();
		}
	}
}

void Actor::bbStand() {
	// make sure that animation was triggered and do nothing
	if (mem_.didBasicBehaviourChange()) {
		animation_control_ptr_->start(AnimationType::ANIMATION_STAND, mem_.getTimeCurrent());
	}
}

void Actor::bbAlignToTarget() {
	HUBERO_LOG("[CAUTION] AlignToTarget behaviour is currently not supported!\r\n");
}

void Actor::bbMoveToGoal() {
	if (mem_.didBasicBehaviourChange()) {
		animation_control_ptr_->start(AnimationType::ANIMATION_WALK, mem_.getTimeCurrent());
		navigation_ptr_->setGoal(mem_.getPoseGoal());
	}
	// retrieve current pose from internal memory
	auto pose = mem_.getPoseCurrent();

	// get command calculated by navigation and convert it to a global coordinate system
	auto cmd_local = navigation_ptr_->getVelocityCmd();
	auto cmd_global = computeCommandToGlobalCs(pose.Rot().Yaw(), cmd_local);
	auto dt = Time::computeDuration(mem_.getTimePrevious(), mem_.getTimeCurrent()).getTime();
	auto path_global_int = cmd_global * dt;
	auto pose_new = Pose3(
		pose.Pos().X() + path_global_int.X(),
		pose.Pos().Y() + path_global_int.Y(),
		pose.Pos().Z(),
		pose.Rot().Roll(),
		pose.Rot().Pitch(),
		pose.Rot().Yaw() + path_global_int.Z()
	);

	// update pose in the internal memory
	mem_.setPose(pose_new);
}

void Actor::bbChooseNewGoal() {
	if (mem_.didBasicBehaviourChange()) {
		animation_control_ptr_->start(AnimationType::ANIMATION_STAND, mem_.getTimeCurrent());
	}
	navigation_ptr_->isPoseAchievable(mem_.getPoseCurrent(), Pose3(Vector3(2.0, 3.0, 0.0), Quaternion()));
}

void Actor::bbAwaitObjectMovement() {
	if (mem_.didBasicBehaviourChange()) {
		animation_control_ptr_->start(AnimationType::ANIMATION_STAND, mem_.getTimeCurrent());
	}
}

void Actor::bbLieDown() {
	if (mem_.didBasicBehaviourChange()) {
		animation_control_ptr_->start(AnimationType::ANIMATION_LIE_DOWN, mem_.getTimeCurrent());
	}
}

void Actor::bbStandUpFromLying() {
	auto pose_adjust = mem_.getPoseCurrent();
	animation_control_ptr_->adjustPose(pose_adjust, mem_.getTimeCurrent());
	mem_.setPose(pose_adjust);
}

void Actor::bbSitDown() {

}

void Actor::bbStandUpFromSitting() {

}

void Actor::bbRun() {

}

void Actor::bbTalk() {

}

void Actor::bbTeleop() {
}

void Actor::updateFsmSuper() {
	EventFsmSuper event {};
	event.follow_object = TaskPredicates(*task_follow_object_ptr_);
	event.lie_down = TaskPredicates(*task_lie_down_ptr_);
	event.move_around = TaskPredicates(*task_move_around_ptr_);
	event.move_to_goal = TaskPredicates(*task_move_to_goal_ptr_);
	event.run = TaskPredicates(*task_run_ptr_);
	event.sit_down = TaskPredicates(*task_sit_down_ptr_);
	event.stand = TaskPredicates(*task_stand_ptr_);
	event.talk = TaskPredicates(*task_talk_ptr_);
	event.teleop = TaskPredicates(*task_teleop_ptr_);
	fsm_.process_event(event);
}

} // namespace hubero
