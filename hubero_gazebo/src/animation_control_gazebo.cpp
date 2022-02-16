#include <hubero_gazebo/animation_control_gazebo.h>

namespace hubero {

const std::map<AnimationType, std::string> AnimationControlGazebo::animation_name_map_ = {
	{ANIMATION_STAND, "stand"},
	{ANIMATION_WALK, "walk"},
	// lying is equal to standing but in a different plane
	{ANIMATION_LIE_DOWN, "stand"},
	{ANIMATION_SIT_DOWN, "sit_down"},
	{ANIMATION_SITTING, "sitting"},
	{ANIMATION_STAND_UP, "stand_up"},
	{ANIMATION_RUN, "run"},
	{ANIMATION_TALK, "talk_a"}
};

AnimationControlGazebo::AnimationControlGazebo():
	AnimationControlBase::AnimationControlBase(),
	animation_configured_recently_(false),
	animation_height_initial_(1.0),
	standing_height_(1.0) {}

void AnimationControlGazebo::initialize(
	std::function<void(gazebo::physics::TrajectoryInfoPtr&)> anim_updater,
	const gazebo::physics::Actor::SkeletonAnimation_M& anims,
	const AnimationType& anim_init,
	const double& standing_height
) {
	// add supported animations handlers
	addAnimationHandler(ANIMATION_STAND, std::bind(&AnimationControlGazebo::handlerStand, this));
	addAnimationHandler(ANIMATION_WALK, std::bind(&AnimationControlGazebo::handlerWalk, this));
	addAnimationHandler(ANIMATION_LIE_DOWN, std::bind(&AnimationControlGazebo::handlerLieDown, this), Time(1.5));
	addAnimationHandler(ANIMATION_SIT_DOWN, std::bind(&AnimationControlGazebo::handlerSitDown, this), Time(1.5));
	addAnimationHandler(ANIMATION_SITTING, std::bind(&AnimationControlGazebo::handlerSitting, this));
	addAnimationHandler(ANIMATION_STAND_UP, std::bind(&AnimationControlGazebo::handlerStandUp, this));
	addAnimationHandler(ANIMATION_RUN, std::bind(&AnimationControlGazebo::handlerRun, this));
	addAnimationHandler(ANIMATION_TALK, std::bind(&AnimationControlGazebo::handlerTalk, this));

	trajectory_updater_ = anim_updater;
	standing_height_ = standing_height;
	skeleton_anims_ = anims;

	// configure
	setupAnimation(anim_init);
}

void AnimationControlGazebo::adjustPose(Pose3& pose, const Time& time_current) {
	if (animation_configured_recently_) {
		animation_height_initial_ = pose.Pos().Z();
		animation_configured_recently_ = false;
	}

	Vector3 pos = pose.Pos();
	Vector3 rpy = pose.Rot().Euler();

	if (getActiveAnimation() == ANIMATION_LIE_DOWN) {
		rpy.X(-IGN_PI / 2);
		pos.Z(0.1);
	} else if (getActiveAnimation() == ANIMATION_SIT_DOWN || getActiveAnimation() == ANIMATION_STAND_UP) {
		Time time_so_far = Time::computeDuration(time_begin_, time_current);
		Time time_range = Time::computeDuration(time_begin_, time_finish_);

		// animation execution progress based on the start and end time stamps
		double time_progress = time_so_far.getTime() / time_range.getTime();

		// trim
		if (time_progress > 1.0) {
			HUBERO_LOG("[AnimationControlGazebo] Animation time progressed out of bounds (%2.3f)\r\n", time_progress);
			time_progress = 1.0;
			anim_finished_ = true;
		}

		// handle each case separately
		if (getActiveAnimation() == ANIMATION_SIT_DOWN) {
			pos.Z(animation_height_initial_ - time_progress * 0.3);
		} else if (getActiveAnimation() == ANIMATION_STAND_UP) {
			pos.Z(animation_height_initial_ - time_progress * 0.2);
		}
	} else {
		// i.e. stand, run etc.
		pos.Z(standing_height_);
		rpy.X(0.0);
	}

	pose.Pos() = pos;
	pose.Rot() = Quaternion(rpy);
}

void AnimationControlGazebo::setupAnimation(AnimationType animation_type) {
	std::string animation;

	auto it = animation_name_map_.find(animation_type);
	if (it != animation_name_map_.end()) {
		animation = it->second;
	} else {
		HUBERO_LOG(
			"[AnimationControlGazebo] Cannot setup animation %d since its literal was not defined\r\n",
			animation_type
		);
		return;
	}

	/* To print available animations:
	for (auto& x: skeleton_anims_) { std::cout << "Skel. animation: " << x.first << std::endl; }
	*/

	if (skeleton_anims_.find(animation) == skeleton_anims_.end()) {
		std::cout << "[AnimationControlGazebo] Skeleton animation ''" << animation << "'' not found.\n";
		return;
	}

	// compute animation duration
	Time duration = Time::computeDuration(time_begin_, time_finish_);

	// create custom trajectory
	trajectory_info_ptr_.reset(new gazebo::physics::TrajectoryInfo());
	trajectory_info_ptr_->type = animation;
	trajectory_info_ptr_->duration = duration.getTime();

	animation_configured_recently_ = true;

	// apply new trajectory
	trajectory_updater_(trajectory_info_ptr_);
}

void AnimationControlGazebo::handlerStand() {
	setupAnimation(ANIMATION_STAND);
}

void AnimationControlGazebo::handlerWalk() {
	setupAnimation(ANIMATION_WALK);
}

void AnimationControlGazebo::handlerLieDown() {
	setupAnimation(ANIMATION_LIE_DOWN);
}

void AnimationControlGazebo::handlerSitDown() {
	setupAnimation(ANIMATION_SIT_DOWN);
}

void AnimationControlGazebo::handlerSitting() {
	setupAnimation(ANIMATION_SITTING);
}

void AnimationControlGazebo::handlerStandUp() {
	setupAnimation(ANIMATION_STAND_UP);
}

void AnimationControlGazebo::handlerRun() {
	setupAnimation(ANIMATION_RUN);
}

void AnimationControlGazebo::handlerTalk() {
	setupAnimation(ANIMATION_TALK);
}

} // namespace hubero
