# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/eero/tello_ros_ws/src/tello_ros/tello_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eero/tello_ros_ws/build/tello_driver

# Include any dependencies generated for this target.
include CMakeFiles/tello_emulator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tello_emulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tello_emulator.dir/flags.make

CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o: CMakeFiles/tello_emulator.dir/flags.make
CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o: /home/eero/tello_ros_ws/src/tello_ros/tello_driver/src/tello_emulator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eero/tello_ros_ws/build/tello_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o -c /home/eero/tello_ros_ws/src/tello_ros/tello_driver/src/tello_emulator.cpp

CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eero/tello_ros_ws/src/tello_ros/tello_driver/src/tello_emulator.cpp > CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.i

CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eero/tello_ros_ws/src/tello_ros/tello_driver/src/tello_emulator.cpp -o CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.s

# Object files for target tello_emulator
tello_emulator_OBJECTS = \
"CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o"

# External object files for target tello_emulator
tello_emulator_EXTERNAL_OBJECTS =

tello_emulator: CMakeFiles/tello_emulator.dir/src/tello_emulator.cpp.o
tello_emulator: CMakeFiles/tello_emulator.dir/build.make
tello_emulator: CMakeFiles/tello_emulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eero/tello_ros_ws/build/tello_driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tello_emulator"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tello_emulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tello_emulator.dir/build: tello_emulator

.PHONY : CMakeFiles/tello_emulator.dir/build

CMakeFiles/tello_emulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tello_emulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tello_emulator.dir/clean

CMakeFiles/tello_emulator.dir/depend:
	cd /home/eero/tello_ros_ws/build/tello_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eero/tello_ros_ws/src/tello_ros/tello_driver /home/eero/tello_ros_ws/src/tello_ros/tello_driver /home/eero/tello_ros_ws/build/tello_driver /home/eero/tello_ros_ws/build/tello_driver /home/eero/tello_ros_ws/build/tello_driver/CMakeFiles/tello_emulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tello_emulator.dir/depend

