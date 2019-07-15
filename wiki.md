* every actor placed in the world MUST use the same plugin (if not using script); otherwise there will likely gazebo errors like `Error in 'gzserver': malloc(): memory corruption: xyz` occur

`<plugin name="actor1" filename="libactor_plugin_social.so">`

`...`

`<plugin name="actorN" filename="libactor_plugin_social.so">`

***

* new animation + probably .so link fix:
https://github.com/DavidPL1/gazebo_ros_actor_cmd_plugin/tree/master/gazebo_ros_actor_cmd_plugin/res

***

* https://bitbucket.org/osrf/gazebo/commits/3794eac

***

If Eclipse doesn't recognize ignition library then add the following external include paths:

`/usr/include/ignition/math3`

`/usr/include/ignition/transport3`

`/usr/include/ignition/msgs0`

And as an include directive use this:

`#include <ignition/math.hh>`

Then in Eclipse run the following:

`Index->Rebuild `

and optionally 

`Index->Freshen All Files`

***

o when Eclipse STOPPED recognizing symbols and `Rebuild` or `Freshen All Files` doesn't help then create a new folder, place some source file in there and run `Rbuild` again (or `Update with modified files`) - now the Indexer should run properly and everything should be fixed

o another way is to try to rebuild from a folder that is higher up in a folders tree

o other solution includes cleaning projects first and then rebuilding all of them (cleaning either with catkin or Eclipse)

o it may be that somehow project's `build` folder in `[workspace]/build` somehow changed ownership and group, then revert changes to default user 

o what helps in most cases is creation of a new workspace in the IDE and firing up the C++ indexer again

o see `~/scripts` folder for solution when Eclipse is stuck at boot


***

`gzserver: /var/lib/jenkins/workspace/gazebo8-debbuilder/build/gazebo-8.6.0/gazebo/common/SkeletonAnimation.cc:147: ignition::math::Matrix4d gazebo::common::NodeAnimation::FrameAt(double, bool) const: Assertion '(t >= 0.0 && t <= 1.0)&&("t is not in the range 0.0..1.0")' failed.
Aborted (core dumped)`

Above error seems to be caused by immediate actor's big displacement and as an effect setting a big difference in script time:

`this->actor->SetScriptTime(this->actor->ScriptTime() + (_dist_traveled * this->animation_factor))`

***

NOTE: with circular bounding box of an actor it is visible that when he tries to avoid obstacles his path is often similar to parts of a circle


***

To make Eclipse IDE recognize created new srv files go to:
click on the project with a right hand mouse button -> Properties -> C/C++ Include Paths and Preprocessor Symbols
and Add External Include Path -> Browse -> go to {YOUR WS FOLDER}/devel and click on the "include" folder and choose OK -> OK -> Apply and Close

created services are achieveable via: 
`#include <{PACKAGE DEFINING NEW SRV}/{SRV_NAME}>`

make sure that the files arent broken (symbolic linkage)

***

It seems that calling the service (by client) must be placed in non-const member function (compilation errors when method marked as const)


***

`Timed out waiting for transform from actor1 to world to become available before running costmap, tf error: canTransform: source_frame actor1 does not exist.. canTransform returned after 0.101011 timeout was 0.1.` -> Check params file somewhere in `actor_global_planner/config/global_costmap` .yaml


***

To add new models to Gazebo - edit `/usr/share/gazebo-8/setup.sh` as shown below and place `3dgems models` accordingly:

	export GAZEBO_MASTER_URI=http://localhost:11345
	export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
	export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-8:${GAZEBO_RESOURCE_PATH}
	export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-8/plugins:${GAZEBO_PLUGIN_PATH}
	export GAZEBO_MODEL_PATH=/usr/share/gazebo-8/models:/usr/share/gazebo-8/3dgems_decoration:/usr/share/gazebo-8/3dgems_furniture:${GAZEBO_MODEL_PATH}
	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-8/plugins
	export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
