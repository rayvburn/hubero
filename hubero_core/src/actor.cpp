#include <hubero_core/actor.h>

namespace hubero {

Actor::Actor() {

}

void Actor::initialize(const std::string& agent_name) {
	try {
		navigation_ptr_ = navigation_loader_.createInstance("hubero/NavigationROS");
		ROS_INFO("Successfully loaded plugin hubero/NavigationROS");
		navigation_ptr_->initialize(agent_name);
	} catch (pluginlib::PluginlibException& ex) {
		// handle the class failing to load
		navigation_ptr_.reset();
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
}

} // namespace hubero