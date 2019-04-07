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

o when Eclipse STOPPED recognizing symbols and Rebuild or Freshen All Files doesn't help then create a new folder, place some source file in there and run Rebuild again - now the Indexer should run properly and everything should be fixed
o another way is to try to rebuild from a folder that is higher up in a folders tree

***

`gzserver: /var/lib/jenkins/workspace/gazebo8-debbuilder/build/gazebo-8.6.0/gazebo/common/SkeletonAnimation.cc:147: ignition::math::Matrix4d gazebo::common::NodeAnimation::FrameAt(double, bool) const: Assertion '(t >= 0.0 && t <= 1.0)&&("t is not in the range 0.0..1.0")' failed.
Aborted (core dumped)`

Above error seems to be caused by immediate actor's big displacement and as an effect setting a big difference in script time:

`this->actor->SetScriptTime(this->actor->ScriptTime() + (_dist_traveled * this->animation_factor))`

***

NOTE: with circular bounding box of an actor it is visible that when he tries to avoid obstacles his path is often similar to parts of a circle