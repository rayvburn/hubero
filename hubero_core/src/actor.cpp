#include <hubero_core/actor.h>
#include <hubero_common/logger.h>

namespace hubero {

Actor::Actor():
	actor_sim_name_("unnamed"),
	mem_ptr_(std::make_shared<InternalMemory>()),
	task_stand_ptr_(std::make_shared<TaskStand>()),
	task_move_to_goal_ptr_(std::make_shared<TaskMoveToGoal>()),
	task_move_around_ptr_(std::make_shared<TaskMoveAround>()),
	task_lie_down_ptr_(std::make_shared<TaskLieDown>()),
	task_sit_down_ptr_(std::make_shared<TaskSitDown>()),
	task_follow_object_ptr_(std::make_shared<TaskFollowObject>()),
	task_teleop_ptr_(std::make_shared<TaskTeleop>()),
	task_run_ptr_(std::make_shared<TaskRun>()),
	task_talk_ptr_(std::make_shared<TaskTalk>()),
	task_map_({
		{TaskType::TASK_FOLLOW_OBJECT, task_follow_object_ptr_},
		{TaskType::TASK_LIE_DOWN, task_lie_down_ptr_},
		{TaskType::TASK_MOVE_AROUND, task_move_around_ptr_},
		{TaskType::TASK_MOVE_TO_GOAL, task_move_to_goal_ptr_},
		{TaskType::TASK_RUN, task_run_ptr_},
		{TaskType::TASK_SIT_DOWN, task_sit_down_ptr_},
		{TaskType::TASK_STAND, task_stand_ptr_},
		{TaskType::TASK_TALK, task_talk_ptr_},
		{TaskType::TASK_TELEOP, task_teleop_ptr_}
	})
{
	TaskBase::addBasicBehaviourHandler(BB_STAND, std::bind(&Actor::bbStand, this));
	TaskBase::addBasicBehaviourHandler(BB_ALIGN_TO_TARGET, std::bind(&Actor::bbAlignToTarget, this));
	TaskBase::addBasicBehaviourHandler(BB_MOVE_TO_GOAL, std::bind(&Actor::bbMoveToGoal, this));
	TaskBase::addBasicBehaviourHandler(BB_CHOOSE_NEW_GOAL, std::bind(&Actor::bbChooseNewGoal, this));
	TaskBase::addBasicBehaviourHandler(BB_FOLLOW_OBJECT, std::bind(&Actor::bbFollowObject, this));
	TaskBase::addBasicBehaviourHandler(BB_LIE_DOWN, std::bind(&Actor::bbLieDown, this));
	TaskBase::addBasicBehaviourHandler(BB_LIE, std::bind(&Actor::bbLie, this));
	TaskBase::addBasicBehaviourHandler(BB_STAND_UP_FROM_LYING, std::bind(&Actor::bbStandUpFromLying, this));
	TaskBase::addBasicBehaviourHandler(BB_SIT_DOWN, std::bind(&Actor::bbSitDown, this));
	TaskBase::addBasicBehaviourHandler(BB_SIT, std::bind(&Actor::bbSit, this));
	TaskBase::addBasicBehaviourHandler(BB_STAND_UP_FROM_SITTING, std::bind(&Actor::bbStandUpFromSitting, this));
	TaskBase::addBasicBehaviourHandler(BB_RUN, std::bind(&Actor::bbRun, this));
	TaskBase::addBasicBehaviourHandler(BB_TALK, std::bind(&Actor::bbTalk, this));
	TaskBase::addBasicBehaviourHandler(BB_TELEOP, std::bind(&Actor::bbTeleop, this));
}

void Actor::initialize(
	const std::string& actor_sim_name,
	std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr,
	std::shared_ptr<hubero::ModelControlBase> model_control_ptr,
	std::shared_ptr<hubero::WorldGeometryBase> world_geometry_ptr,
	std::shared_ptr<hubero::LocalisationBase> localisation_ptr,
	std::shared_ptr<hubero::NavigationBase> navigation_ptr,
	std::shared_ptr<hubero::StatusBase> status_ptr,
	std::shared_ptr<hubero::TaskRequestBase> task_request_ptr
) {
	actor_sim_name_ = actor_sim_name;

	animation_control_ptr_ = animation_control_ptr;
	model_control_ptr_ = model_control_ptr;
	world_geometry_ptr_ = world_geometry_ptr;
	localisation_ptr_  = localisation_ptr;
	navigation_ptr_ = navigation_ptr;
	status_ptr_ = status_ptr;
	task_request_ptr_ = task_request_ptr;

	// check if valid pointers were given
	if (!isInitialized()) {
		HUBERO_LOG("Actor class initialization failure\r\n");
		return;
	}

	// initialize tasks
	task_stand_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_move_to_goal_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_move_around_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_lie_down_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_sit_down_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_follow_object_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_teleop_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_run_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);
	task_talk_ptr_->initialize(animation_control_ptr_, navigation_ptr_, world_geometry_ptr_, mem_ptr_);

	// initialize map with task executors
	state_tf_exec_map_ = decltype(state_tf_exec_map_)({
		{FsmSuper::State::STAND, std::bind(&TaskStand::execute, task_stand_ptr_)},
		{FsmSuper::State::MOVE_TO_GOAL, std::bind(&TaskMoveToGoal::execute, task_move_to_goal_ptr_)},
		{FsmSuper::State::MOVE_AROUND, std::bind(&TaskMoveAround::execute, task_move_around_ptr_)},
		{FsmSuper::State::LIE_DOWN, std::bind(&TaskLieDown::execute, task_lie_down_ptr_)},
		{FsmSuper::State::SIT_DOWN, std::bind(&TaskSitDown::execute, task_sit_down_ptr_)},
		{FsmSuper::State::FOLLOW_OBJECT, std::bind(&TaskFollowObject::execute, task_follow_object_ptr_)},
		{FsmSuper::State::TELEOP, std::bind(&TaskTeleop::execute, task_teleop_ptr_)},
		{FsmSuper::State::RUN, std::bind(&TaskRun::execute, task_run_ptr_)},
		{FsmSuper::State::TALK, std::bind(&TaskTalk::execute, task_talk_ptr_)}
	});

	// add tasks that are requestable
	task_request_ptr_->addTask(task_stand_ptr_->getTaskType(), task_stand_ptr_);
	task_request_ptr_->addTask(task_move_to_goal_ptr_->getTaskType(), task_move_to_goal_ptr_);
	task_request_ptr_->addTask(task_move_around_ptr_->getTaskType(), task_move_around_ptr_);
	task_request_ptr_->addTask(task_lie_down_ptr_->getTaskType(), task_lie_down_ptr_);
	task_request_ptr_->addTask(task_sit_down_ptr_->getTaskType(), task_sit_down_ptr_);
	task_request_ptr_->addTask(task_follow_object_ptr_->getTaskType(), task_follow_object_ptr_);
	task_request_ptr_->addTask(task_teleop_ptr_->getTaskType(), task_teleop_ptr_);
	task_request_ptr_->addTask(task_run_ptr_->getTaskType(), task_run_ptr_);
	task_request_ptr_->addTask(task_talk_ptr_->getTaskType(), task_talk_ptr_);

	// name is set, update FSM logger preamble
	fsm_.setLoggerPreamble(actor_sim_name_);

	// navigation_ptr_ updated with a valid object, so FsmSuper's transition handlers can be updated
	Actor::addFsmSuperTransitionHandlers(
		fsm_,
		task_stand_ptr_,
		task_move_to_goal_ptr_,
		task_move_around_ptr_,
		task_lie_down_ptr_,
		task_sit_down_ptr_,
		task_follow_object_ptr_,
		task_teleop_ptr_,
		task_run_ptr_,
		task_talk_ptr_,
		navigation_ptr_
	);
}

void Actor::update(const Time& time) {
	if (!isInitialized()) {
		HUBERO_LOG("Actor class not initialized yet!\r\n");
		return;
	}

	// input data
	mem_ptr_->setTime(time);

	// pose post-processing for smooth animation (this is specific to implementation and simulator)
	auto pose_adjusted = localisation_ptr_->getPose();
	animation_control_ptr_->adjustPose(pose_adjusted, mem_ptr_->getTimeCurrent());
	// NOTE: InternalMemory::setPose updates history, so it cannot be called twice - otherwise displacement will be 0
	mem_ptr_->setPose(pose_adjusted);

	// execute transition function of the specific task and update FSM predicates
	auto tf_exec_it = state_tf_exec_map_.find(static_cast<FsmSuper::State>(fsm_.current_state()));
	if (tf_exec_it != state_tf_exec_map_.end()) {
		tf_exec_it->second();
	} else {
		HUBERO_LOG("FsmSuper has a state that Actor class is not aware of! Cannot execute transition function\r\n");
	}

	// output data
	localisation_ptr_->update(mem_ptr_->getPoseCurrent());

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

	status_ptr_->update(
		localisation_ptr_->getPose(),
		localisation_ptr_->getVelocityLinear(),
		localisation_ptr_->getVelocityAngular()
	);

	// Task-related FSMs got updated at the end of transition function execution so now update highest level FSM
	updateFsmSuper();
}

bool Actor::isInitialized() const {
	return animation_control_ptr_ != nullptr
		&& model_control_ptr_ != nullptr
		&& world_geometry_ptr_ != nullptr
		&& localisation_ptr_ != nullptr
		&& navigation_ptr_ != nullptr
		&& task_request_ptr_ != nullptr;
}

// static
void Actor::addFsmSuperTransitionHandlers(
	FsmSuper& fsm,
	std::shared_ptr<TaskStand> task_stand_ptr,
	std::shared_ptr<TaskMoveToGoal> task_move_to_goal_ptr,
	std::shared_ptr<TaskMoveAround> task_move_around_ptr,
	std::shared_ptr<TaskLieDown> task_lie_down_ptr,
	std::shared_ptr<TaskSitDown> task_sit_down_ptr,
	std::shared_ptr<TaskFollowObject> task_follow_object_ptr,
	std::shared_ptr<TaskTeleop> task_teleop_ptr,
	std::shared_ptr<TaskRun> task_run_ptr,
	std::shared_ptr<TaskTalk> task_talk_ptr,
	std::shared_ptr<NavigationBase> navigation_ptr
) {
	// terminate
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::MOVE_TO_GOAL, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::MOVE_AROUND, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::FOLLOW_OBJECT, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::LIE_DOWN, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::SIT_DOWN, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::RUN, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::TALK, std::bind(&TaskStand::terminate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::TELEOP, std::bind(&TaskStand::terminate, task_stand_ptr));

	fsm.addTransitionHandler(FsmSuper::State::MOVE_TO_GOAL, FsmSuper::State::STAND, std::bind(&TaskMoveToGoal::terminate, task_move_to_goal_ptr));
	fsm.addTransitionHandler(FsmSuper::State::MOVE_AROUND, FsmSuper::State::STAND, std::bind(&TaskMoveAround::terminate, task_move_around_ptr));
	fsm.addTransitionHandler(FsmSuper::State::FOLLOW_OBJECT, FsmSuper::State::STAND, std::bind(&TaskFollowObject::terminate, task_follow_object_ptr));
	fsm.addTransitionHandler(FsmSuper::State::LIE_DOWN, FsmSuper::State::STAND, std::bind(&TaskLieDown::terminate, task_lie_down_ptr));
	fsm.addTransitionHandler(FsmSuper::State::SIT_DOWN, FsmSuper::State::STAND, std::bind(&TaskSitDown::terminate, task_sit_down_ptr));
	fsm.addTransitionHandler(FsmSuper::State::RUN, FsmSuper::State::STAND, std::bind(&TaskRun::terminate, task_run_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TALK, FsmSuper::State::STAND, std::bind(&TaskTalk::terminate, task_talk_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TELEOP, FsmSuper::State::STAND, std::bind(&TaskTeleop::terminate, task_teleop_ptr));

	// activate
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::MOVE_TO_GOAL, std::bind(&TaskMoveToGoal::activate, task_move_to_goal_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::MOVE_AROUND, std::bind(&TaskMoveAround::activate, task_move_around_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::FOLLOW_OBJECT, std::bind(&TaskFollowObject::activate, task_follow_object_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::LIE_DOWN, std::bind(&TaskLieDown::activate, task_lie_down_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::SIT_DOWN, std::bind(&TaskSitDown::activate, task_sit_down_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::RUN, std::bind(&TaskRun::activate, task_run_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::TALK, std::bind(&TaskTalk::activate, task_talk_ptr));
	fsm.addTransitionHandler(FsmSuper::State::STAND, FsmSuper::State::TELEOP, std::bind(&TaskTeleop::activate, task_teleop_ptr));

	fsm.addTransitionHandler(FsmSuper::State::MOVE_TO_GOAL, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::MOVE_AROUND, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::FOLLOW_OBJECT, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::LIE_DOWN, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::SIT_DOWN, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::RUN, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TALK, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TELEOP, FsmSuper::State::STAND, std::bind(&TaskStand::activate, task_stand_ptr));

	// finish navigation
	fsm.addTransitionHandler(FsmSuper::State::MOVE_TO_GOAL, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::MOVE_AROUND, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::FOLLOW_OBJECT, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::LIE_DOWN, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::SIT_DOWN, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::RUN, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TALK, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
	fsm.addTransitionHandler(FsmSuper::State::TELEOP, FsmSuper::State::STAND, std::bind(&NavigationBase::finish, navigation_ptr));
}

// static
Pose3 Actor::computeNewPose(const Pose3& pose_current, const Vector3& cmd_vel, const Time& dt) {
	// process velocity command - compute displacement
	auto path_global_int = cmd_vel * dt.getTime();
	auto pose_new = Pose3(
		pose_current.Pos().X() + path_global_int.X(),
		pose_current.Pos().Y() + path_global_int.Y(),
		pose_current.Pos().Z(),
		pose_current.Rot().Roll(),
		pose_current.Rot().Pitch(),
		pose_current.Rot().Yaw() + path_global_int.Z()
	);
	return pose_new;
}

void Actor::bbStand() {
}

void Actor::bbAlignToTarget() {
	HUBERO_LOG("[CAUTION] AlignToTarget behaviour is currently not supported!\r\n");
}

void Actor::bbMoveToGoal() {
	// process navigation command - compute displacement
	auto pose_new = Actor::computeNewPose(
		mem_ptr_->getPoseCurrent(),
		navigation_ptr_->getVelocityCmd(),
		Time::computeDuration(mem_ptr_->getTimePrevious(), mem_ptr_->getTimeCurrent()).getTime()
	);

	// update pose in the internal memory
	mem_ptr_->setPose(pose_new);
}

void Actor::bbFollowObject() {
	// evaluate, if plan is outdated and re-generation is required
	if (mem_ptr_->getTimeSinceLastGoalUpdate() >= GOAL_UPDATE_PERIOD_DEFAULT) {
		auto goal_pose = navigation_ptr_->computeClosestAchievablePose(
			mem_ptr_->getPoseGoal(),
			navigation_ptr_->getWorldFrame()
		);
		mem_ptr_->setGoal(goal_pose);
		mem_ptr_->setGoalPoseUpdateTime(mem_ptr_->getTimeCurrent());
		navigation_ptr_->setGoal(mem_ptr_->getPoseGoal(), navigation_ptr_->getWorldFrame());
	}

	// process navigation command - compute displacement
	auto pose_new = Actor::computeNewPose(
		mem_ptr_->getPoseCurrent(),
		navigation_ptr_->getVelocityCmd(),
		Time::computeDuration(mem_ptr_->getTimePrevious(), mem_ptr_->getTimeCurrent()).getTime()
	);

	// update pose in the internal memory
	mem_ptr_->setPose(pose_new);
}

void Actor::bbChooseNewGoal() {
	// do not trigger planning too often
	if (mem_ptr_->getTimeSinceLastGoalUpdate() <= CHOOSE_NEW_GOAL_RETRY_PERIOD_DEFAULT) {
		return;
	}

	// find a new goal
	bool goal_valid = false;
	Pose3 goal;
	std::tie(goal_valid, goal) = navigation_ptr_->findRandomReachableGoal();
	if (goal_valid) {
		navigation_ptr_->setGoal(goal, navigation_ptr_->getGlobalReferenceFrame());
		mem_ptr_->setGoal(goal);
		mem_ptr_->setGoalPoseUpdateTime(mem_ptr_->getTimeCurrent());
	}
}

void Actor::bbLieDown() {
}

void Actor::bbLie() {
}

void Actor::bbStandUpFromLying() {
}

void Actor::bbSitDown() {
}

void Actor::bbSit() {
}

void Actor::bbStandUpFromSitting() {
}

void Actor::bbRun() {
	// process navigation command - compute displacement
	// FIXME: note the hack for faster movement!
	auto pose_new = Actor::computeNewPose(
		mem_ptr_->getPoseCurrent(),
		Vector3(
			2.0 * navigation_ptr_->getVelocityCmd().X(),
			navigation_ptr_->getVelocityCmd().Y(),
			navigation_ptr_->getVelocityCmd().Z()
		),
		Time::computeDuration(mem_ptr_->getTimePrevious(), mem_ptr_->getTimeCurrent()).getTime()
	);

	// update pose in the internal memory
	mem_ptr_->setPose(pose_new);
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
