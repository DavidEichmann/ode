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
CMAKE_SOURCE_DIR = /home/david/git/ODE_01/ODE_01/lib/ogre-procedural

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/git/ODE_01/ODE_01/lib/ogre-procedural

# Include any dependencies generated for this target.
include samples/primitives/CMakeFiles/Sample_Primitives.dir/depend.make

# Include the progress variables for this target.
include samples/primitives/CMakeFiles/Sample_Primitives.dir/progress.make

# Include the compile flags for this target's objects.
include samples/primitives/CMakeFiles/Sample_Primitives.dir/flags.make

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o: samples/primitives/CMakeFiles/Sample_Primitives.dir/flags.make
samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o: samples/common/src/BaseApplication.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o -c /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/common/src/BaseApplication.cpp

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.i"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/common/src/BaseApplication.cpp > CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.i

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.s"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/common/src/BaseApplication.cpp -o CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.s

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.requires:
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.requires

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.provides: samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.requires
	$(MAKE) -f samples/primitives/CMakeFiles/Sample_Primitives.dir/build.make samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.provides.build
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.provides

samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.provides.build: samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o: samples/primitives/CMakeFiles/Sample_Primitives.dir/flags.make
samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o: samples/primitives/src/Primitives.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o -c /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives/src/Primitives.cpp

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.i"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives/src/Primitives.cpp > CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.i

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.s"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives/src/Primitives.cpp -o CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.s

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.requires:
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.requires

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.provides: samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.requires
	$(MAKE) -f samples/primitives/CMakeFiles/Sample_Primitives.dir/build.make samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.provides.build
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.provides

samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.provides.build: samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o

# Object files for target Sample_Primitives
Sample_Primitives_OBJECTS = \
"CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o" \
"CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o"

# External object files for target Sample_Primitives
Sample_Primitives_EXTERNAL_OBJECTS =

bin/Sample_Primitives: samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o
bin/Sample_Primitives: samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o
bin/Sample_Primitives: samples/primitives/CMakeFiles/Sample_Primitives.dir/build.make
bin/Sample_Primitives: lib/libOgreProcedural.so
bin/Sample_Primitives: /usr/lib/i386-linux-gnu/libOgreMain.so
bin/Sample_Primitives: /usr/lib/i386-linux-gnu/libOIS.so
bin/Sample_Primitives: samples/primitives/CMakeFiles/Sample_Primitives.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/Sample_Primitives"
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Sample_Primitives.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/primitives/CMakeFiles/Sample_Primitives.dir/build: bin/Sample_Primitives
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/build

samples/primitives/CMakeFiles/Sample_Primitives.dir/requires: samples/primitives/CMakeFiles/Sample_Primitives.dir/__/common/src/BaseApplication.cpp.o.requires
samples/primitives/CMakeFiles/Sample_Primitives.dir/requires: samples/primitives/CMakeFiles/Sample_Primitives.dir/src/Primitives.cpp.o.requires
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/requires

samples/primitives/CMakeFiles/Sample_Primitives.dir/clean:
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives && $(CMAKE_COMMAND) -P CMakeFiles/Sample_Primitives.dir/cmake_clean.cmake
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/clean

samples/primitives/CMakeFiles/Sample_Primitives.dir/depend:
	cd /home/david/git/ODE_01/ODE_01/lib/ogre-procedural && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/git/ODE_01/ODE_01/lib/ogre-procedural /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives /home/david/git/ODE_01/ODE_01/lib/ogre-procedural /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives /home/david/git/ODE_01/ODE_01/lib/ogre-procedural/samples/primitives/CMakeFiles/Sample_Primitives.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/primitives/CMakeFiles/Sample_Primitives.dir/depend

