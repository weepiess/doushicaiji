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
CMAKE_SOURCE_DIR = /home/wyx/程序/wyx/doushicaiji

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wyx/程序/wyx/doushicaiji/build

# Include any dependencies generated for this target.
include libBase/CMakeFiles/Base.dir/depend.make

# Include the progress variables for this target.
include libBase/CMakeFiles/Base.dir/progress.make

# Include the compile flags for this target's objects.
include libBase/CMakeFiles/Base.dir/flags.make

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o: libBase/CMakeFiles/Base.dir/flags.make
libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o: ../libBase/src/base_thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wyx/程序/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Base.dir/src/base_thread.cpp.o -c /home/wyx/程序/wyx/doushicaiji/libBase/src/base_thread.cpp

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Base.dir/src/base_thread.cpp.i"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wyx/程序/wyx/doushicaiji/libBase/src/base_thread.cpp > CMakeFiles/Base.dir/src/base_thread.cpp.i

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Base.dir/src/base_thread.cpp.s"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wyx/程序/wyx/doushicaiji/libBase/src/base_thread.cpp -o CMakeFiles/Base.dir/src/base_thread.cpp.s

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.requires:

.PHONY : libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.requires

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.provides: libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.requires
	$(MAKE) -f libBase/CMakeFiles/Base.dir/build.make libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.provides.build
.PHONY : libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.provides

libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.provides.build: libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o


libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o: libBase/CMakeFiles/Base.dir/flags.make
libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o: ../libBase/src/basic_tool.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wyx/程序/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Base.dir/src/basic_tool.cpp.o -c /home/wyx/程序/wyx/doushicaiji/libBase/src/basic_tool.cpp

libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Base.dir/src/basic_tool.cpp.i"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wyx/程序/wyx/doushicaiji/libBase/src/basic_tool.cpp > CMakeFiles/Base.dir/src/basic_tool.cpp.i

libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Base.dir/src/basic_tool.cpp.s"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wyx/程序/wyx/doushicaiji/libBase/src/basic_tool.cpp -o CMakeFiles/Base.dir/src/basic_tool.cpp.s

libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.requires:

.PHONY : libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.requires

libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.provides: libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.requires
	$(MAKE) -f libBase/CMakeFiles/Base.dir/build.make libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.provides.build
.PHONY : libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.provides

libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.provides.build: libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o


libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o: libBase/CMakeFiles/Base.dir/flags.make
libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o: ../libBase/src/image_tool.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wyx/程序/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Base.dir/src/image_tool.cpp.o -c /home/wyx/程序/wyx/doushicaiji/libBase/src/image_tool.cpp

libBase/CMakeFiles/Base.dir/src/image_tool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Base.dir/src/image_tool.cpp.i"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wyx/程序/wyx/doushicaiji/libBase/src/image_tool.cpp > CMakeFiles/Base.dir/src/image_tool.cpp.i

libBase/CMakeFiles/Base.dir/src/image_tool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Base.dir/src/image_tool.cpp.s"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wyx/程序/wyx/doushicaiji/libBase/src/image_tool.cpp -o CMakeFiles/Base.dir/src/image_tool.cpp.s

libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.requires:

.PHONY : libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.requires

libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.provides: libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.requires
	$(MAKE) -f libBase/CMakeFiles/Base.dir/build.make libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.provides.build
.PHONY : libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.provides

libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.provides.build: libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o


# Object files for target Base
Base_OBJECTS = \
"CMakeFiles/Base.dir/src/base_thread.cpp.o" \
"CMakeFiles/Base.dir/src/basic_tool.cpp.o" \
"CMakeFiles/Base.dir/src/image_tool.cpp.o"

# External object files for target Base
Base_EXTERNAL_OBJECTS =

libBase/libBase.a: libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o
libBase/libBase.a: libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o
libBase/libBase.a: libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o
libBase/libBase.a: libBase/CMakeFiles/Base.dir/build.make
libBase/libBase.a: libBase/CMakeFiles/Base.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wyx/程序/wyx/doushicaiji/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libBase.a"
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && $(CMAKE_COMMAND) -P CMakeFiles/Base.dir/cmake_clean_target.cmake
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Base.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libBase/CMakeFiles/Base.dir/build: libBase/libBase.a

.PHONY : libBase/CMakeFiles/Base.dir/build

libBase/CMakeFiles/Base.dir/requires: libBase/CMakeFiles/Base.dir/src/base_thread.cpp.o.requires
libBase/CMakeFiles/Base.dir/requires: libBase/CMakeFiles/Base.dir/src/basic_tool.cpp.o.requires
libBase/CMakeFiles/Base.dir/requires: libBase/CMakeFiles/Base.dir/src/image_tool.cpp.o.requires

.PHONY : libBase/CMakeFiles/Base.dir/requires

libBase/CMakeFiles/Base.dir/clean:
	cd /home/wyx/程序/wyx/doushicaiji/build/libBase && $(CMAKE_COMMAND) -P CMakeFiles/Base.dir/cmake_clean.cmake
.PHONY : libBase/CMakeFiles/Base.dir/clean

libBase/CMakeFiles/Base.dir/depend:
	cd /home/wyx/程序/wyx/doushicaiji/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wyx/程序/wyx/doushicaiji /home/wyx/程序/wyx/doushicaiji/libBase /home/wyx/程序/wyx/doushicaiji/build /home/wyx/程序/wyx/doushicaiji/build/libBase /home/wyx/程序/wyx/doushicaiji/build/libBase/CMakeFiles/Base.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libBase/CMakeFiles/Base.dir/depend

