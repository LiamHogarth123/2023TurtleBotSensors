# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/liam/git/2023TurtleBotSensors/turtleboi

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liam/git/2023TurtleBotSensors/turtleboi/build

# Include any dependencies generated for this target.
include CMakeFiles/turtleboi_sensorprocessing.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/turtleboi_sensorprocessing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/turtleboi_sensorprocessing.dir/flags.make

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o: CMakeFiles/turtleboi_sensorprocessing.dir/flags.make
CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o: ../src/sensorprocessing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liam/git/2023TurtleBotSensors/turtleboi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o -c /home/liam/git/2023TurtleBotSensors/turtleboi/src/sensorprocessing.cpp

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liam/git/2023TurtleBotSensors/turtleboi/src/sensorprocessing.cpp > CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.i

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liam/git/2023TurtleBotSensors/turtleboi/src/sensorprocessing.cpp -o CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.s

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.requires:

.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.requires

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.provides: CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.requires
	$(MAKE) -f CMakeFiles/turtleboi_sensorprocessing.dir/build.make CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.provides.build
.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.provides

CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.provides.build: CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o


# Object files for target turtleboi_sensorprocessing
turtleboi_sensorprocessing_OBJECTS = \
"CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o"

# External object files for target turtleboi_sensorprocessing
turtleboi_sensorprocessing_EXTERNAL_OBJECTS =

devel/lib/libturtleboi_sensorprocessing.so: CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o
devel/lib/libturtleboi_sensorprocessing.so: CMakeFiles/turtleboi_sensorprocessing.dir/build.make
devel/lib/libturtleboi_sensorprocessing.so: CMakeFiles/turtleboi_sensorprocessing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liam/git/2023TurtleBotSensors/turtleboi/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/libturtleboi_sensorprocessing.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/turtleboi_sensorprocessing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/turtleboi_sensorprocessing.dir/build: devel/lib/libturtleboi_sensorprocessing.so

.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/build

CMakeFiles/turtleboi_sensorprocessing.dir/requires: CMakeFiles/turtleboi_sensorprocessing.dir/src/sensorprocessing.cpp.o.requires

.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/requires

CMakeFiles/turtleboi_sensorprocessing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/turtleboi_sensorprocessing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/clean

CMakeFiles/turtleboi_sensorprocessing.dir/depend:
	cd /home/liam/git/2023TurtleBotSensors/turtleboi/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liam/git/2023TurtleBotSensors/turtleboi /home/liam/git/2023TurtleBotSensors/turtleboi /home/liam/git/2023TurtleBotSensors/turtleboi/build /home/liam/git/2023TurtleBotSensors/turtleboi/build /home/liam/git/2023TurtleBotSensors/turtleboi/build/CMakeFiles/turtleboi_sensorprocessing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/turtleboi_sensorprocessing.dir/depend
