# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rkelbs/mekabot/meka-ros-pkg/m3_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build

# Include any dependencies generated for this target.
include CMakeFiles/loadx6_client_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/loadx6_client_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/loadx6_client_example.dir/flags.make

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: CMakeFiles/loadx6_client_example.dir/flags.make
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: ../src/examples/loadx6_client_example.cpp
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: ../manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/genmsg_cpp/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/tools/rospack/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roslib/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/rosconsole/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/std_msgs/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roslang/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/rospy/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/3rdparty/xmlrpcpp/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roscpp/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/stacks/geometry/eigen/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/stacks/geometry/kdl/manifest.xml
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roslib/msg_gen/generated
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/std_msgs/msg_gen/generated
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roscpp/msg_gen/generated
CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o: /opt/ros/cturtle/ros/core/roscpp/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DEIGEN_USE_NEW_STDVECTOR -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o -c /home/rkelbs/mekabot/meka-ros-pkg/m3_client/src/examples/loadx6_client_example.cpp

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DEIGEN_USE_NEW_STDVECTOR -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/rkelbs/mekabot/meka-ros-pkg/m3_client/src/examples/loadx6_client_example.cpp > CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.i

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DEIGEN_USE_NEW_STDVECTOR -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/rkelbs/mekabot/meka-ros-pkg/m3_client/src/examples/loadx6_client_example.cpp -o CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.s

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.requires:
.PHONY : CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.requires

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.provides: CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.requires
	$(MAKE) -f CMakeFiles/loadx6_client_example.dir/build.make CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.provides.build
.PHONY : CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.provides

CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.provides.build: CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o
.PHONY : CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.provides.build

# Object files for target loadx6_client_example
loadx6_client_example_OBJECTS = \
"CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o"

# External object files for target loadx6_client_example
loadx6_client_example_EXTERNAL_OBJECTS =

../bin/loadx6_client_example: CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o
../bin/loadx6_client_example: ../lib/libm3_client.so
../bin/loadx6_client_example: CMakeFiles/loadx6_client_example.dir/build.make
../bin/loadx6_client_example: CMakeFiles/loadx6_client_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/loadx6_client_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/loadx6_client_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/loadx6_client_example.dir/build: ../bin/loadx6_client_example
.PHONY : CMakeFiles/loadx6_client_example.dir/build

CMakeFiles/loadx6_client_example.dir/requires: CMakeFiles/loadx6_client_example.dir/src/examples/loadx6_client_example.o.requires
.PHONY : CMakeFiles/loadx6_client_example.dir/requires

CMakeFiles/loadx6_client_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/loadx6_client_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/loadx6_client_example.dir/clean

CMakeFiles/loadx6_client_example.dir/depend:
	cd /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rkelbs/mekabot/meka-ros-pkg/m3_client /home/rkelbs/mekabot/meka-ros-pkg/m3_client /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build /home/rkelbs/mekabot/meka-ros-pkg/m3_client/build/CMakeFiles/loadx6_client_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/loadx6_client_example.dir/depend

