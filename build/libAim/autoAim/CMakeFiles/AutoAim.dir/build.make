# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/two/RM2019/wyx/doushicaiji

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/two/RM2019/wyx/doushicaiji/build

# Include any dependencies generated for this target.
include libAim/autoAim/CMakeFiles/AutoAim.dir/depend.make

# Include the progress variables for this target.
include libAim/autoAim/CMakeFiles/AutoAim.dir/progress.make

# Include the compile flags for this target's objects.
include libAim/autoAim/CMakeFiles/AutoAim.dir/flags.make

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o: libAim/autoAim/CMakeFiles/AutoAim.dir/flags.make
libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o: ../libAim/autoAim/src/kalman_filter_by_opencv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/two/RM2019/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o -c /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/kalman_filter_by_opencv.cpp

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.i"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/kalman_filter_by_opencv.cpp > CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.i

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.s"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/kalman_filter_by_opencv.cpp -o CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.s

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.requires:

.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.requires

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.provides: libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.requires
	$(MAKE) -f libAim/autoAim/CMakeFiles/AutoAim.dir/build.make libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.provides.build
.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.provides

libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.provides.build: libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o


libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o: libAim/autoAim/CMakeFiles/AutoAim.dir/flags.make
libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o: ../libAim/autoAim/src/auto_aim.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/two/RM2019/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o -c /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/auto_aim.cpp

libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AutoAim.dir/src/auto_aim.cpp.i"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/auto_aim.cpp > CMakeFiles/AutoAim.dir/src/auto_aim.cpp.i

libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AutoAim.dir/src/auto_aim.cpp.s"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/two/RM2019/wyx/doushicaiji/libAim/autoAim/src/auto_aim.cpp -o CMakeFiles/AutoAim.dir/src/auto_aim.cpp.s

libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.requires:

.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.requires

libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.provides: libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.requires
	$(MAKE) -f libAim/autoAim/CMakeFiles/AutoAim.dir/build.make libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.provides.build
.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.provides

libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.provides.build: libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o


# Object files for target AutoAim
AutoAim_OBJECTS = \
"CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o" \
"CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o"

# External object files for target AutoAim
AutoAim_EXTERNAL_OBJECTS =

libAim/autoAim/libAutoAim.a: libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o
libAim/autoAim/libAutoAim.a: libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o
libAim/autoAim/libAutoAim.a: libAim/autoAim/CMakeFiles/AutoAim.dir/build.make
libAim/autoAim/libAutoAim.a: libAim/autoAim/CMakeFiles/AutoAim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/two/RM2019/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libAutoAim.a"
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && $(CMAKE_COMMAND) -P CMakeFiles/AutoAim.dir/cmake_clean_target.cmake
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AutoAim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libAim/autoAim/CMakeFiles/AutoAim.dir/build: libAim/autoAim/libAutoAim.a

.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/build

libAim/autoAim/CMakeFiles/AutoAim.dir/requires: libAim/autoAim/CMakeFiles/AutoAim.dir/src/kalman_filter_by_opencv.cpp.o.requires
libAim/autoAim/CMakeFiles/AutoAim.dir/requires: libAim/autoAim/CMakeFiles/AutoAim.dir/src/auto_aim.cpp.o.requires

.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/requires

libAim/autoAim/CMakeFiles/AutoAim.dir/clean:
	cd /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim && $(CMAKE_COMMAND) -P CMakeFiles/AutoAim.dir/cmake_clean.cmake
.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/clean

libAim/autoAim/CMakeFiles/AutoAim.dir/depend:
	cd /home/two/RM2019/wyx/doushicaiji/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/two/RM2019/wyx/doushicaiji /home/two/RM2019/wyx/doushicaiji/libAim/autoAim /home/two/RM2019/wyx/doushicaiji/build /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim /home/two/RM2019/wyx/doushicaiji/build/libAim/autoAim/CMakeFiles/AutoAim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libAim/autoAim/CMakeFiles/AutoAim.dir/depend

