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
CMAKE_SOURCE_DIR = /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build

# Include any dependencies generated for this target.
include CMakeFiles/test_jsonize.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_jsonize.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_jsonize.dir/flags.make

CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o: CMakeFiles/test_jsonize.dir/flags.make
CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o: ../example/jsonize/test_jsonize.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o -c /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/example/jsonize/test_jsonize.cpp

CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/example/jsonize/test_jsonize.cpp > CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.i

CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/example/jsonize/test_jsonize.cpp -o CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.s

# Object files for target test_jsonize
test_jsonize_OBJECTS = \
"CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o"

# External object files for target test_jsonize
test_jsonize_EXTERNAL_OBJECTS =

test_jsonize: CMakeFiles/test_jsonize.dir/example/jsonize/test_jsonize.cpp.o
test_jsonize: CMakeFiles/test_jsonize.dir/build.make
test_jsonize: CMakeFiles/test_jsonize.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_jsonize"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_jsonize.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_jsonize.dir/build: test_jsonize

.PHONY : CMakeFiles/test_jsonize.dir/build

CMakeFiles/test_jsonize.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_jsonize.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_jsonize.dir/clean

CMakeFiles/test_jsonize.dir/depend:
	cd /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2 /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2 /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build /home/zhenghao/walk-these-ways-go2/go2_gym_deploy/unitree_sdk2_bin/library/unitree_sdk2/build/CMakeFiles/test_jsonize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_jsonize.dir/depend

