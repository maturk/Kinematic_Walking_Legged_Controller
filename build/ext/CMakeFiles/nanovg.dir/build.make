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
include ext/CMakeFiles/nanovg.dir/depend.make

# Include the progress variables for this target.
include ext/CMakeFiles/nanovg.dir/progress.make

# Include the compile flags for this target's objects.
include ext/CMakeFiles/nanovg.dir/flags.make

ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o: ext/CMakeFiles/nanovg.dir/flags.make
ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o: _deps/nanovg-src/src/nanovg.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o"
	cd /home/maturk/git/CMM/a2-maturk/build/ext && /usr/bin/x86_64-linux-gnu-gcc-9 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o   -c /home/maturk/git/CMM/a2-maturk/build/_deps/nanovg-src/src/nanovg.c

ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.i"
	cd /home/maturk/git/CMM/a2-maturk/build/ext && /usr/bin/x86_64-linux-gnu-gcc-9 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/maturk/git/CMM/a2-maturk/build/_deps/nanovg-src/src/nanovg.c > CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.i

ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.s"
	cd /home/maturk/git/CMM/a2-maturk/build/ext && /usr/bin/x86_64-linux-gnu-gcc-9 $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/maturk/git/CMM/a2-maturk/build/_deps/nanovg-src/src/nanovg.c -o CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.s

# Object files for target nanovg
nanovg_OBJECTS = \
"CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o"

# External object files for target nanovg
nanovg_EXTERNAL_OBJECTS =

ext/libnanovg.a: ext/CMakeFiles/nanovg.dir/__/_deps/nanovg-src/src/nanovg.c.o
ext/libnanovg.a: ext/CMakeFiles/nanovg.dir/build.make
ext/libnanovg.a: ext/CMakeFiles/nanovg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maturk/git/CMM/a2-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libnanovg.a"
	cd /home/maturk/git/CMM/a2-maturk/build/ext && $(CMAKE_COMMAND) -P CMakeFiles/nanovg.dir/cmake_clean_target.cmake
	cd /home/maturk/git/CMM/a2-maturk/build/ext && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nanovg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ext/CMakeFiles/nanovg.dir/build: ext/libnanovg.a

.PHONY : ext/CMakeFiles/nanovg.dir/build

ext/CMakeFiles/nanovg.dir/clean:
	cd /home/maturk/git/CMM/a2-maturk/build/ext && $(CMAKE_COMMAND) -P CMakeFiles/nanovg.dir/cmake_clean.cmake
.PHONY : ext/CMakeFiles/nanovg.dir/clean

ext/CMakeFiles/nanovg.dir/depend:
	cd /home/maturk/git/CMM/a2-maturk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maturk/git/CMM/a2-maturk /home/maturk/git/CMM/a2-maturk/ext /home/maturk/git/CMM/a2-maturk/build /home/maturk/git/CMM/a2-maturk/build/ext /home/maturk/git/CMM/a2-maturk/build/ext/CMakeFiles/nanovg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ext/CMakeFiles/nanovg.dir/depend

