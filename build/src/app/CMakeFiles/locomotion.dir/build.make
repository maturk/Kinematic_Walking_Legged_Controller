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
include src/app/CMakeFiles/locomotion.dir/depend.make

# Include the progress variables for this target.
include src/app/CMakeFiles/locomotion.dir/progress.make

# Include the compile flags for this target's objects.
include src/app/CMakeFiles/locomotion.dir/flags.make

src/app/CMakeFiles/locomotion.dir/main.cpp.o: src/app/CMakeFiles/locomotion.dir/flags.make
src/app/CMakeFiles/locomotion.dir/main.cpp.o: ../src/app/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/app/CMakeFiles/locomotion.dir/main.cpp.o"
	cd /home/maturk/git/CMM/a2-maturk/build/src/app && /usr/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/locomotion.dir/main.cpp.o -c /home/maturk/git/CMM/a2-maturk/src/app/main.cpp

src/app/CMakeFiles/locomotion.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/locomotion.dir/main.cpp.i"
	cd /home/maturk/git/CMM/a2-maturk/build/src/app && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maturk/git/CMM/a2-maturk/src/app/main.cpp > CMakeFiles/locomotion.dir/main.cpp.i

src/app/CMakeFiles/locomotion.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/locomotion.dir/main.cpp.s"
	cd /home/maturk/git/CMM/a2-maturk/build/src/app && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maturk/git/CMM/a2-maturk/src/app/main.cpp -o CMakeFiles/locomotion.dir/main.cpp.s

# Object files for target locomotion
locomotion_OBJECTS = \
"CMakeFiles/locomotion.dir/main.cpp.o"

# External object files for target locomotion
locomotion_EXTERNAL_OBJECTS =

src/app/locomotion: src/app/CMakeFiles/locomotion.dir/main.cpp.o
src/app/locomotion: src/app/CMakeFiles/locomotion.dir/build.make
src/app/locomotion: src/libs/utils/libutils.a
src/app/locomotion: src/libs/gui/libgui.a
src/app/locomotion: src/libs/simAndControl/libsimAndControl.a
src/app/locomotion: src/libs/gui/libgui.a
src/app/locomotion: src/libs/utils/libutils.a
src/app/locomotion: ext/glad/libglad.a
src/app/locomotion: /usr/lib/x86_64-linux-gnu/libOpenGL.so
src/app/locomotion: /usr/lib/x86_64-linux-gnu/libGLX.so
src/app/locomotion: /usr/lib/x86_64-linux-gnu/libGLU.so
src/app/locomotion: ext/ext/glfw/src/libglfw3.a
src/app/locomotion: /usr/lib/x86_64-linux-gnu/librt.so
src/app/locomotion: /usr/lib/x86_64-linux-gnu/libm.so
src/app/locomotion: /usr/lib/x86_64-linux-gnu/libX11.so
src/app/locomotion: ext/libimgui.a
src/app/locomotion: ext/tinyobjloader/libtinyobjloader.a
src/app/locomotion: src/app/CMakeFiles/locomotion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable locomotion"
	cd /home/maturk/git/CMM/a2-maturk/build/src/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/locomotion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/app/CMakeFiles/locomotion.dir/build: src/app/locomotion

.PHONY : src/app/CMakeFiles/locomotion.dir/build

src/app/CMakeFiles/locomotion.dir/clean:
	cd /home/maturk/git/CMM/a2-maturk/build/src/app && $(CMAKE_COMMAND) -P CMakeFiles/locomotion.dir/cmake_clean.cmake
.PHONY : src/app/CMakeFiles/locomotion.dir/clean

src/app/CMakeFiles/locomotion.dir/depend:
	cd /home/maturk/git/CMM/a2-maturk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maturk/git/CMM/a2-maturk /home/maturk/git/CMM/a2-maturk/src/app /home/maturk/git/CMM/a2-maturk/build /home/maturk/git/CMM/a2-maturk/build/src/app /home/maturk/git/CMM/a2-maturk/build/src/app/CMakeFiles/locomotion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/app/CMakeFiles/locomotion.dir/depend

