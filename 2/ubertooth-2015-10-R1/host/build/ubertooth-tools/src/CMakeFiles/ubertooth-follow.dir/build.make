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
CMAKE_SOURCE_DIR = /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build

# Include any dependencies generated for this target.
include ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/depend.make

# Include the progress variables for this target.
include ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/progress.make

# Include the compile flags for this target's objects.
include ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/flags.make

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/flags.make
ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o: ../ubertooth-tools/src/ubertooth-follow.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o"
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o   -c /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/ubertooth-tools/src/ubertooth-follow.c

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.i"
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/ubertooth-tools/src/ubertooth-follow.c > CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.i

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.s"
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/ubertooth-tools/src/ubertooth-follow.c -o CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.s

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.requires:
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.requires

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.provides: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.requires
	$(MAKE) -f ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/build.make ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.provides.build
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.provides

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.provides.build: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o

# Object files for target ubertooth-follow
ubertooth__follow_OBJECTS = \
"CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o"

# External object files for target ubertooth-follow
ubertooth__follow_EXTERNAL_OBJECTS =

ubertooth-tools/src/ubertooth-follow: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o
ubertooth-tools/src/ubertooth-follow: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/build.make
ubertooth-tools/src/ubertooth-follow: libubertooth/src/libubertooth.so.0.2
ubertooth-tools/src/ubertooth-follow: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
ubertooth-tools/src/ubertooth-follow: /usr/local/lib/libbtbb.so
ubertooth-tools/src/ubertooth-follow: /usr/lib/x86_64-linux-gnu/libbluetooth.so
ubertooth-tools/src/ubertooth-follow: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C executable ubertooth-follow"
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ubertooth-follow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/build: ubertooth-tools/src/ubertooth-follow
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/build

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/requires: ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/ubertooth-follow.c.o.requires
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/requires

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/clean:
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src && $(CMAKE_COMMAND) -P CMakeFiles/ubertooth-follow.dir/cmake_clean.cmake
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/clean

ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/depend:
	cd /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/ubertooth-tools/src /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src /home/weiping/wpson-ubertooth/ubertooth-2015-10-R1/host/build/ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-follow.dir/depend

