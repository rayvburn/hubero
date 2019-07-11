Gazebo GDB

## Based on `https://www.ceh-photo.de/blog/?p=899`, but this targets Gazebo Server application (and its libraries/plugins) in particular.

## 1) `-GEclipse CDT4 - Unix Makefiles -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -D__cplusplus=201103L -D__GXX_EXPERIMENTAL_CXX0X__=1`
  Change the above `catkin config` to:
`catkin config -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER_ARG1=-std=c++11 -D__cplusplus=201103L -D__GXX_EXPERIMENTAL_CXX0X__=1`

## 2) Clean and build again the workspace:
`catkin clean` (prompt will ask whether to delete files - OK)
`catkin build`

## 3) Change your base node's (`gzserver` in this case) `.launch` file - `launch-prefix` has to be added `launch-prefix="xterm -e gdbserver localhost:10000"` which will start new instance of `xterm`.

## 4) Start the Eclipse binary from a bash session that has sourced the relevant setup.bash file (https://answers.ros.org/question/248045/file-optroskineticbinroslaunch-line-34-in-module-importerror-no-module-named-roslaunch/).

## 5) * NOT TESTED * , lack of PyDev; Eclipse seems to overwrite some Python env variable?
Take a view on `Window > Preferences > PyDev > Interpreter Python ...` You can use `Auto Config` or declare the path manually. (https://answers.ros.org/question/10043/how-do-i-start-a-launch-file-from-eclipse-ide/)

## 6) In Eclipse choose `Run->Debug Configurations`. The currently edited project should appear there. ??? IT HAS TO BE THE PLUGIN? not PLUGIN's LIBRARY?
## 7) Choose `C/C++ Remote Application -> Main`. In the `C/C++ Application` textbox paste a path to gzserver (`/usr/bin/gzserver`) ???? THIS DOES NOT WORK


##### `Run` works properly (https://answers.ros.org/question/10043/how-do-i-start-a-launch-file-from-eclipse-ide/) but Debug does not
##### Even when C/C++ application is set to:
`/opt/ros/kinetic/bin/roslaunch` and `Arguments` are `actor_sim_utils example_full.launch world:=living_room_vid_friendly gdb:=true`