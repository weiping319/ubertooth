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
CMAKE_SOURCE_DIR = /home/weiping/wpson-ubertooth/coordinate/host

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weiping/wpson-ubertooth/coordinate/host/libubertooth

# Include any dependencies generated for this target.
include ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/depend.make

# Include the progress variables for this target.
include ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/progress.make

# Include the compile flags for this target's objects.
include ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/flags.make

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/flags.make
ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o: ../ubertooth-tools/src/ubertooth-specan.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o"
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o   -c /home/weiping/wpson-ubertooth/coordinate/host/ubertooth-tools/src/ubertooth-specan.c

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.i"
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/weiping/wpson-ubertooth/coordinate/host/ubertooth-tools/src/ubertooth-specan.c > CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.i

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.s"
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/weiping/wpson-ubertooth/coordinate/host/ubertooth-tools/src/ubertooth-specan.c -o CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.s

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.requires:
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.requires

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.provides: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.requires
	$(MAKE) -f ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/build.make ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.provides.build
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.provides

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.provides.build: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o

# Object files for target ubertooth-specan
ubertooth__specan_OBJECTS = \
"CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o"

# External object files for target ubertooth-specan
ubertooth__specan_EXTERNAL_OBJECTS =

ubertooth-tools/src/ubertooth-specan: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o
ubertooth-tools/src/ubertooth-specan: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/build.make
ubertooth-tools/src/ubertooth-specan: libubertooth/src/libubertooth.so.0.2
ubertooth-tools/src/ubertooth-specan: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
ubertooth-tools/src/ubertooth-specan: /usr/local/lib/libbtbb.so
ubertooth-tools/src/ubertooth-specan: /usr/lib/x86_64-linux-gnu/libbluetooth.so
ubertooth-tools/src/ubertooth-specan: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C executable ubertooth-specan"
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ubertooth-specan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/build: ubertooth-tools/src/ubertooth-specan
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/build

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/requires: ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/ubertooth-specan.c.o.requires
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/requires

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/clean:
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src && $(CMAKE_COMMAND) -P CMakeFiles/ubertooth-specan.dir/cmake_clean.cmake
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/clean

ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/depend:
	cd /home/weiping/wpson-ubertooth/coordinate/host/libubertooth && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weiping/wpson-ubertooth/coordinate/host /home/weiping/wpson-ubertooth/coordinate/host/ubertooth-tools/src /home/weiping/wpson-ubertooth/coordinate/host/libubertooth /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src /home/weiping/wpson-ubertooth/coordinate/host/libubertooth/ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ubertooth-tools/src/CMakeFiles/ubertooth-specan.dir/depend

