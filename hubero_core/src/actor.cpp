#include <hubero_core/actor.h>
#include <hubero_common/logger.h>

namespace hubero {

Actor::Actor(): actor_sim_name_("unnamed") {

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
