#include <hubero_gazebo/animation_control_gazebo.h>

namespace hubero {

const std::map<AnimationType, std::string> AnimationControlGazebo::animation_name_map_ = {
	{ANIMATION_STAND, "stand"},
	{ANIMATION_WALK, "walk"},
	// lying is equal to standing but in a different plane
	{ANIMATION_LIE_DOWN, "stand"},
	{ANIMATION_LYING, "stand"},
	{ANIMATION_SIT_DOWN, "sit_down"},
	{ANIMATION_SITTING, "sitting"},
	{ANIMATION_STAND_UP, "stand_up"},
	{ANIMATION_RUN, "run"},
	{ANIMATION_TALK, "talk_a"}
};

AnimationControlGazebo::AnimationControlGazebo():
	AnimationControlBase::AnimationControlBase(),
	animation_configured_recently_(false),
	animation_pose_initial_(Pose3(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)),
	standing_height_(1.0) {}

void AnimationControlGazebo::initialize(
	std::function<void(gazebo::physics::TrajectoryInfoPtr&)> anim_updater,
	const gazebo::physics::Actor::SkeletonAnimation_M& anims,
	const AnimationType& anim_init,
	const double& standing_height
) {
	// add supported animations handlers
	addAnimationHandler(
		ANIMATION_STAND,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_STAND)
	);
	addAnimationHandler(
		ANIMATION_WALK,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_WALK)
	);
	addAnimationHandler(
		ANIMATION_LIE_DOWN,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_LIE_DOWN),
		Time(1.5)
	);
	addAnimationHandler(
		ANIMATION_LYING,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_LYING)
	);
	addAnimationHandler(
		ANIMATION_SIT_DOWN,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_SIT_DOWN),
		Time(1.5)
	);
	addAnimationHandler(
		ANIMATION_SITTING,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_SITTING)
	);
	addAnimationHandler(
		ANIMATION_STAND_UP,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_STAND_UP),
		Time(1.5)
	);
	addAnimationHandler(
		ANIMATION_RUN,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_RUN)
	);
	addAnimationHandler(
		ANIMATION_TALK,
		std::bind(&AnimationControlGazebo::setupAnimation, this, ANIMATION_TALK)
	);

	trajectory_updater_ = anim_updater;
	standing_height_ = standing_height;
	skeleton_anims_ = anims;

	// configure
	setupAnimation(anim_init);
}

void AnimationControlGazebo::adjustPose(Pose3& pose, const Time& time_current) {
	if (animation_configured_recently_) {
		animation_pose_initial_ = pose;
		animation_configured_recently_ = false;
	}

	Vector3 pos = pose.Pos();
	Vector3 rpy = pose.Rot().Euler();

	if (getActiveAnimation() == ANIMATION_LYING) {
		rpy.X(ROT_ROLL_LYING_GROUND);
		pos.Z(standing_height_ + POS_Z_LYING_GROUND_DELTA);
	} else if (getActiveAnimation() == ANIMATION_SITTING) {
		rpy.X(ROT_ROLL_STANDING);
		pos.Z(standing_height_ + POS_Z_SITTING_DELTA);
	} else if (
		getActiveAnimation() == ANIMATION_WALK
		|| getActiveAnimation() == ANIMATION_RUN
		|| getActiveAnimation() == ANIMATION_STAND
		|| getActiveAnimation() == ANIMATION_TALK
	) {
		// i.e. stand, run etc.
		pos.Z(standing_height_);
		rpy.X(0.0);
	} else if (
		getActiveAnimation() == ANIMATION_SIT_DOWN
		|| getActiveAnimation() == ANIMATION_LIE_DOWN
		|| getActiveAnimation() == ANIMATION_STAND_UP
	) {
		Time time_so_far = Time::computeDuration(time_begin_, time_current);
		Time time_range = Time::computeDuration(time_begin_, time_finish_);

		// animation execution progress based on the start and end time stamps
		double time_progress = time_so_far.getTime() / time_range.getTime();

		// trim
		if (time_progress >= 1.0) {
			HUBERO_LOG("[AnimationControlGazebo] Animation time progressed out of bounds (%2.3f)\r\n", time_progress);
			time_progress = 1.0;
			anim_finished_ = true;
		}

		// handle each case separately
		if (getActiveAnimation() == ANIMATION_SIT_DOWN) {
			pos.Z(animation_pose_initial_.Pos().Z() + time_progress * POS_Z_SITTING_DELTA);
		} else if (getActiveAnimation() == ANIMATION_LIE_DOWN) {
			rpy.X(ROT_ROLL_STANDING + ROT_ROLL_LYING_GROUND * time_progress);
			double pos_z_diff = (standing_height_ + POS_Z_LYING_GROUND_DELTA)  - animation_pose_initial_.Pos().Z();
			pos.Z(standing_height_ + (time_progress * pos_z_diff));
		} else if (getActiveAnimation() == ANIMATION_STAND_UP) {
			double rot_roll_diff = ROT_ROLL_STANDING - animation_pose_initial_.Rot().Roll();
			double pos_z_diff = standing_height_ - animation_pose_initial_.Pos().Z();
			rpy.X(animation_pose_initial_.Rot().X() + (time_progress * rot_roll_diff));
			pos.Z(animation_pose_initial_.Pos().Z() + (time_progress * pos_z_diff));
		}
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

} // namespace hubero
