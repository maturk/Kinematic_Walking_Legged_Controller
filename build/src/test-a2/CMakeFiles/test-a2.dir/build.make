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
CMAKE_SOURCE_DIR = /home/maturk/git/CMM/a2-maturk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maturk/git/CMM/a2-maturk/build

# Include any dependencies generated for this target.
include src/test-a2/CMakeFiles/test-a2.dir/depend.make

# Include the progress variables for this target.
include src/test-a2/CMakeFiles/test-a2.dir/progress.make

# Include the compile flags for this target's objects.
include src/test-a2/CMakeFiles/test-a2.dir/flags.make

src/test-a2/CMakeFiles/test-a2.dir/test.cpp.o: src/test-a2/CMakeFiles/test-a2.dir/flags.make
src/test-a2/CMakeFiles/test-a2.dir/test.cpp.o: ../src/test-a2/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/test-a2/CMakeFiles/test-a2.dir/test.cpp.o"
	cd /home/maturk/git/CMM/a2-maturk/build/src/test-a2 && /usr/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test-a2.dir/test.cpp.o -c /home/maturk/git/CMM/a2-maturk/src/test-a2/test.cpp

src/test-a2/CMakeFiles/test-a2.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-a2.dir/test.cpp.i"
	cd /home/maturk/git/CMM/a2-maturk/build/src/test-a2 && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maturk/git/CMM/a2-maturk/src/test-a2/test.cpp > CMakeFiles/test-a2.dir/test.cpp.i

src/test-a2/CMakeFiles/test-a2.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-a2.dir/test.cpp.s"
	cd /home/maturk/git/CMM/a2-maturk/build/src/test-a2 && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maturk/git/CMM/a2-maturk/src/test-a2/test.cpp -o CMakeFiles/test-a2.dir/test.cpp.s

# Object files for target test-a2
test__a2_OBJECTS = \
"CMakeFiles/test-a2.dir/test.cpp.o"

# External object files for target test-a2
test__a2_EXTERNAL_OBJECTS =

src/test-a2/test-a2: src/test-a2/CMakeFiles/test-a2.dir/test.cpp.o
src/test-a2/test-a2: src/test-a2/CMakeFiles/test-a2.dir/build.make
src/test-a2/test-a2: src/libs/utils/libutils.a
src/test-a2/test-a2: src/libs/simAndControl/libsimAndControl.a
src/test-a2/test-a2: src/libs/gui/libgui.a
src/test-a2/test-a2: src/libs/utils/libutils.a
src/test-a2/test-a2: ext/glad/libglad.a
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/libOpenGL.so
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/libGLX.so
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/libGLU.so
src/test-a2/test-a2: ext/ext/glfw/src/libglfw3.a
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/librt.so
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/libm.so
src/test-a2/test-a2: /usr/lib/x86_64-linux-gnu/libX11.so
src/test-a2/test-a2: ext/libimgui.a
src/test-a2/test-a2: ext/tinyobjloader/libtinyobjloader.a
src/test-a2/test-a2: src/test-a2/CMakeFiles/test-a2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test-a2"
	cd /home/maturk/git/CMM/a2-maturk/build/src/test-a2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-a2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/test-a2/CMakeFiles/test-a2.dir/build: src/test-a2/test-a2

.PHONY : src/test-a2/CMakeFiles/test-a2.dir/build

src/test-a2/CMakeFiles/test-a2.dir/clean:
	cd /home/maturk/git/CMM/a2-maturk/build/src/test-a2 && $(CMAKE_COMMAND) -P CMakeFiles/test-a2.dir/cmake_clean.cmake
.PHONY : src/test-a2/CMakeFiles/test-a2.dir/clean

src/test-a2/CMakeFiles/test-a2.dir/depend:
	cd /home/maturk/git/CMM/a2-maturk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maturk/git/CMM/a2-maturk /home/maturk/git/CMM/a2-maturk/src/test-a2 /home/maturk/git/CMM/a2-maturk/build /home/maturk/git/CMM/a2-maturk/build/src/test-a2 /home/maturk/git/CMM/a2-maturk/build/src/test-a2/CMakeFiles/test-a2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/test-a2/CMakeFiles/test-a2.dir/depend

