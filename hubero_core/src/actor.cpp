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
}

void Actor::update(const Time& time) {
	if (!isInitialized()) {
		HUBERO_LOG("Actor class not initialized yet!\r\n");
		return;
	}
}

bool Actor::isInitialized() const {
	return animation_control_ptr_ != nullptr
		&& model_control_ptr_ != nullptr
		&& localisation_ptr_ != nullptr
		&& navigation_ptr_ != nullptr
		&& task_request_ptr_ != nullptr;
}

void Actor::bbStand() {

}

void Actor::bbAlignToTarget() {

}

void Actor::bbMoveToGoal() {

}

void Actor::bbChooseNewGoal() {

}

void Actor::bbAwaitObjectMovement() {

}

void Actor::bbLieDown() {

}

void Actor::bbStandUpFromLying() {

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

} // namespace hubero
