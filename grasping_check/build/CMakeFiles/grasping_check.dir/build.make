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
CMAKE_SOURCE_DIR = /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build

# Include any dependencies generated for this target.
include CMakeFiles/grasping_check.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/grasping_check.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/grasping_check.dir/flags.make

CMakeFiles/grasping_check.dir/src/main.cpp.o: CMakeFiles/grasping_check.dir/flags.make
CMakeFiles/grasping_check.dir/src/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/grasping_check.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/grasping_check.dir/src/main.cpp.o -c /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/main.cpp

CMakeFiles/grasping_check.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/grasping_check.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/main.cpp > CMakeFiles/grasping_check.dir/src/main.cpp.i

CMakeFiles/grasping_check.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/grasping_check.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/main.cpp -o CMakeFiles/grasping_check.dir/src/main.cpp.s

CMakeFiles/grasping_check.dir/src/main.cpp.o.requires:
.PHONY : CMakeFiles/grasping_check.dir/src/main.cpp.o.requires

CMakeFiles/grasping_check.dir/src/main.cpp.o.provides: CMakeFiles/grasping_check.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/grasping_check.dir/build.make CMakeFiles/grasping_check.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/grasping_check.dir/src/main.cpp.o.provides

CMakeFiles/grasping_check.dir/src/main.cpp.o.provides.build: CMakeFiles/grasping_check.dir/src/main.cpp.o

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o: CMakeFiles/grasping_check.dir/flags.make
CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o: ../src/grasping_check.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o -c /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/grasping_check.cpp

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/grasping_check.dir/src/grasping_check.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/grasping_check.cpp > CMakeFiles/grasping_check.dir/src/grasping_check.cpp.i

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/grasping_check.dir/src/grasping_check.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/src/grasping_check.cpp -o CMakeFiles/grasping_check.dir/src/grasping_check.cpp.s

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.requires:
.PHONY : CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.requires

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.provides: CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.requires
	$(MAKE) -f CMakeFiles/grasping_check.dir/build.make CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.provides.build
.PHONY : CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.provides

CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.provides.build: CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o

# Object files for target grasping_check
grasping_check_OBJECTS = \
"CMakeFiles/grasping_check.dir/src/main.cpp.o" \
"CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o"

# External object files for target grasping_check
grasping_check_EXTERNAL_OBJECTS =

../bin/grasping_check: CMakeFiles/grasping_check.dir/src/main.cpp.o
../bin/grasping_check: CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o
../bin/grasping_check: CMakeFiles/grasping_check.dir/build.make
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/grasping_check: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/grasping_check: CMakeFiles/grasping_check.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/grasping_check"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/grasping_check.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/grasping_check.dir/build: ../bin/grasping_check
.PHONY : CMakeFiles/grasping_check.dir/build

CMakeFiles/grasping_check.dir/requires: CMakeFiles/grasping_check.dir/src/main.cpp.o.requires
CMakeFiles/grasping_check.dir/requires: CMakeFiles/grasping_check.dir/src/grasping_check.cpp.o.requires
.PHONY : CMakeFiles/grasping_check.dir/requires

CMakeFiles/grasping_check.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/grasping_check.dir/cmake_clean.cmake
.PHONY : CMakeFiles/grasping_check.dir/clean

CMakeFiles/grasping_check.dir/depend:
	cd /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build /home/freelist/RoCKin2015/repo/SPQR_at_work/grasping_check/build/CMakeFiles/grasping_check.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/grasping_check.dir/depend

