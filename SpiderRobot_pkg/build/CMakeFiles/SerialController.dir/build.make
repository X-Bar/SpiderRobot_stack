# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build

# Include any dependencies generated for this target.
include CMakeFiles/SerialController.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SerialController.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SerialController.dir/flags.make

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: CMakeFiles/SerialController.dir/flags.make
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: ../src/SerialController_sub.cpp
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: ../manifest.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/cpp_common/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rostime/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/roscpp_traits/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/roscpp_serialization/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/genmsg/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/genpy/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/message_runtime/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/std_msgs/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rosgraph/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rosgraph_msgs/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/catkin/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rospack/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/roslib/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rospy/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/rosconsole/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/xmlrpcpp/package.xml
CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o: /opt/ros/groovy/share/roscpp/package.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o -c /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/src/SerialController_sub.cpp

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/src/SerialController_sub.cpp > CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.i

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/src/SerialController_sub.cpp -o CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.s

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.requires:
.PHONY : CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.requires

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.provides: CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.requires
	$(MAKE) -f CMakeFiles/SerialController.dir/build.make CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.provides.build
.PHONY : CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.provides

CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.provides.build: CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o

# Object files for target SerialController
SerialController_OBJECTS = \
"CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o"

# External object files for target SerialController
SerialController_EXTERNAL_OBJECTS =

../bin/SerialController: CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o
../bin/SerialController: CMakeFiles/SerialController.dir/build.make
../bin/SerialController: CMakeFiles/SerialController.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/SerialController"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialController.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SerialController.dir/build: ../bin/SerialController
.PHONY : CMakeFiles/SerialController.dir/build

CMakeFiles/SerialController.dir/requires: CMakeFiles/SerialController.dir/src/SerialController_sub.cpp.o.requires
.PHONY : CMakeFiles/SerialController.dir/requires

CMakeFiles/SerialController.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SerialController.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SerialController.dir/clean

CMakeFiles/SerialController.dir/depend:
	cd /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build /home/xbar/groovy_workspace/Git/SpiderRobot_stack/SpiderRobot_pkg/build/CMakeFiles/SerialController.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SerialController.dir/depend

