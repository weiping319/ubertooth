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
CMAKE_SOURCE_DIR = /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth

# Include any dependencies generated for this target.
include libubertooth/src/CMakeFiles/ubertooth.dir/depend.make

# Include the progress variables for this target.
include libubertooth/src/CMakeFiles/ubertooth.dir/progress.make

# Include the compile flags for this target's objects.
include libubertooth/src/CMakeFiles/ubertooth.dir/flags.make

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o: libubertooth/src/CMakeFiles/ubertooth.dir/flags.make
libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o: src/ubertooth.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ubertooth.dir/ubertooth.c.o   -c /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth.c

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ubertooth.dir/ubertooth.c.i"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth.c > CMakeFiles/ubertooth.dir/ubertooth.c.i

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ubertooth.dir/ubertooth.c.s"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth.c -o CMakeFiles/ubertooth.dir/ubertooth.c.s

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.requires:
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.requires

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.provides: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.requires
	$(MAKE) -f libubertooth/src/CMakeFiles/ubertooth.dir/build.make libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.provides.build
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.provides

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.provides.build: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o: libubertooth/src/CMakeFiles/ubertooth.dir/flags.make
libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o: src/ubertooth_control.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/ubertooth.dir/ubertooth_control.c.o   -c /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth_control.c

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ubertooth.dir/ubertooth_control.c.i"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth_control.c > CMakeFiles/ubertooth.dir/ubertooth_control.c.i

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ubertooth.dir/ubertooth_control.c.s"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src/ubertooth_control.c -o CMakeFiles/ubertooth.dir/ubertooth_control.c.s

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.requires:
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.requires

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.provides: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.requires
	$(MAKE) -f libubertooth/src/CMakeFiles/ubertooth.dir/build.make libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.provides.build
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.provides

libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.provides.build: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o

# Object files for target ubertooth
ubertooth_OBJECTS = \
"CMakeFiles/ubertooth.dir/ubertooth.c.o" \
"CMakeFiles/ubertooth.dir/ubertooth_control.c.o"

# External object files for target ubertooth
ubertooth_EXTERNAL_OBJECTS =

libubertooth/src/libubertooth.so.0.2: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o
libubertooth/src/libubertooth.so.0.2: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o
libubertooth/src/libubertooth.so.0.2: libubertooth/src/CMakeFiles/ubertooth.dir/build.make
libubertooth/src/libubertooth.so.0.2: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
libubertooth/src/libubertooth.so.0.2: /usr/local/lib/libbtbb.so
libubertooth/src/libubertooth.so.0.2: libubertooth/src/CMakeFiles/ubertooth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library libubertooth.so"
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ubertooth.dir/link.txt --verbose=$(VERBOSE)
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && $(CMAKE_COMMAND) -E cmake_symlink_library libubertooth.so.0.2 libubertooth.so.0 libubertooth.so

libubertooth/src/libubertooth.so.0: libubertooth/src/libubertooth.so.0.2

libubertooth/src/libubertooth.so: libubertooth/src/libubertooth.so.0.2

# Rule to build all files generated by this target.
libubertooth/src/CMakeFiles/ubertooth.dir/build: libubertooth/src/libubertooth.so
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/build

libubertooth/src/CMakeFiles/ubertooth.dir/requires: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth.c.o.requires
libubertooth/src/CMakeFiles/ubertooth.dir/requires: libubertooth/src/CMakeFiles/ubertooth.dir/ubertooth_control.c.o.requires
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/requires

libubertooth/src/CMakeFiles/ubertooth.dir/clean:
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src && $(CMAKE_COMMAND) -P CMakeFiles/ubertooth.dir/cmake_clean.cmake
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/clean

libubertooth/src/CMakeFiles/ubertooth.dir/depend:
	cd /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/src /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src /home/weiping/wpson-ubertooth/2/ubertooth-2015-10-R1/host/libubertooth/libubertooth/src/CMakeFiles/ubertooth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libubertooth/src/CMakeFiles/ubertooth.dir/depend
