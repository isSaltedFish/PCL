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
CMAKE_SOURCE_DIR = /home/ros/Downloads/exp14

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/Downloads/exp14/build

# Include any dependencies generated for this target.
include CMakeFiles/plygeneration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plygeneration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plygeneration.dir/flags.make

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o: CMakeFiles/plygeneration.dir/flags.make
CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o: ../plyfileGeneration.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ros/Downloads/exp14/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o -c /home/ros/Downloads/exp14/plyfileGeneration.cpp

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ros/Downloads/exp14/plyfileGeneration.cpp > CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.i

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ros/Downloads/exp14/plyfileGeneration.cpp -o CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.s

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.requires:
.PHONY : CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.requires

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.provides: CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.requires
	$(MAKE) -f CMakeFiles/plygeneration.dir/build.make CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.provides.build
.PHONY : CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.provides

CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.provides.build: CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o

CMakeFiles/plygeneration.dir/rply.c.o: CMakeFiles/plygeneration.dir/flags.make
CMakeFiles/plygeneration.dir/rply.c.o: ../rply.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ros/Downloads/exp14/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/plygeneration.dir/rply.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/plygeneration.dir/rply.c.o   -c /home/ros/Downloads/exp14/rply.c

CMakeFiles/plygeneration.dir/rply.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/plygeneration.dir/rply.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/ros/Downloads/exp14/rply.c > CMakeFiles/plygeneration.dir/rply.c.i

CMakeFiles/plygeneration.dir/rply.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/plygeneration.dir/rply.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/ros/Downloads/exp14/rply.c -o CMakeFiles/plygeneration.dir/rply.c.s

CMakeFiles/plygeneration.dir/rply.c.o.requires:
.PHONY : CMakeFiles/plygeneration.dir/rply.c.o.requires

CMakeFiles/plygeneration.dir/rply.c.o.provides: CMakeFiles/plygeneration.dir/rply.c.o.requires
	$(MAKE) -f CMakeFiles/plygeneration.dir/build.make CMakeFiles/plygeneration.dir/rply.c.o.provides.build
.PHONY : CMakeFiles/plygeneration.dir/rply.c.o.provides

CMakeFiles/plygeneration.dir/rply.c.o.provides.build: CMakeFiles/plygeneration.dir/rply.c.o

# Object files for target plygeneration
plygeneration_OBJECTS = \
"CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o" \
"CMakeFiles/plygeneration.dir/rply.c.o"

# External object files for target plygeneration
plygeneration_EXTERNAL_OBJECTS =

plygeneration: CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o
plygeneration: CMakeFiles/plygeneration.dir/rply.c.o
plygeneration: CMakeFiles/plygeneration.dir/build.make
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_system.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plygeneration: /usr/lib/x86_64-linux-gnu/libpthread.so
plygeneration: /usr/lib/libpcl_common.so
plygeneration: /usr/lib/libpcl_octree.so
plygeneration: /usr/lib/libOpenNI.so
plygeneration: /usr/lib/libvtkCommon.so.5.8.0
plygeneration: /usr/lib/libvtkRendering.so.5.8.0
plygeneration: /usr/lib/libvtkHybrid.so.5.8.0
plygeneration: /usr/lib/libvtkCharts.so.5.8.0
plygeneration: /usr/lib/libpcl_io.so
plygeneration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plygeneration: /usr/lib/libpcl_kdtree.so
plygeneration: /usr/lib/libpcl_search.so
plygeneration: /usr/lib/libpcl_sample_consensus.so
plygeneration: /usr/lib/libpcl_filters.so
plygeneration: /usr/lib/libpcl_features.so
plygeneration: /usr/lib/libpcl_keypoints.so
plygeneration: /usr/lib/libpcl_segmentation.so
plygeneration: /usr/lib/libpcl_visualization.so
plygeneration: /usr/lib/libpcl_outofcore.so
plygeneration: /usr/lib/libpcl_registration.so
plygeneration: /usr/lib/libpcl_recognition.so
plygeneration: /usr/lib/x86_64-linux-gnu/libqhull.so
plygeneration: /usr/lib/libpcl_surface.so
plygeneration: /usr/lib/libpcl_people.so
plygeneration: /usr/lib/libpcl_tracking.so
plygeneration: /usr/lib/libpcl_apps.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_system.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_thread.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
plygeneration: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
plygeneration: /usr/lib/x86_64-linux-gnu/libpthread.so
plygeneration: /usr/lib/x86_64-linux-gnu/libqhull.so
plygeneration: /usr/lib/libOpenNI.so
plygeneration: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
plygeneration: /usr/lib/libvtkCommon.so.5.8.0
plygeneration: /usr/lib/libvtkRendering.so.5.8.0
plygeneration: /usr/lib/libvtkHybrid.so.5.8.0
plygeneration: /usr/lib/libvtkCharts.so.5.8.0
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
plygeneration: /usr/lib/libpcl_common.so
plygeneration: /usr/lib/libpcl_octree.so
plygeneration: /usr/lib/libpcl_io.so
plygeneration: /usr/lib/libpcl_kdtree.so
plygeneration: /usr/lib/libpcl_search.so
plygeneration: /usr/lib/libpcl_sample_consensus.so
plygeneration: /usr/lib/libpcl_filters.so
plygeneration: /usr/lib/libpcl_features.so
plygeneration: /usr/lib/libpcl_keypoints.so
plygeneration: /usr/lib/libpcl_segmentation.so
plygeneration: /usr/lib/libpcl_visualization.so
plygeneration: /usr/lib/libpcl_outofcore.so
plygeneration: /usr/lib/libpcl_registration.so
plygeneration: /usr/lib/libpcl_recognition.so
plygeneration: /usr/lib/libpcl_surface.so
plygeneration: /usr/lib/libpcl_people.so
plygeneration: /usr/lib/libpcl_tracking.so
plygeneration: /usr/lib/libpcl_apps.so
plygeneration: /usr/lib/libvtkViews.so.5.8.0
plygeneration: /usr/lib/libvtkInfovis.so.5.8.0
plygeneration: /usr/lib/libvtkWidgets.so.5.8.0
plygeneration: /usr/lib/libvtkHybrid.so.5.8.0
plygeneration: /usr/lib/libvtkParallel.so.5.8.0
plygeneration: /usr/lib/libvtkVolumeRendering.so.5.8.0
plygeneration: /usr/lib/libvtkRendering.so.5.8.0
plygeneration: /usr/lib/libvtkGraphics.so.5.8.0
plygeneration: /usr/lib/libvtkImaging.so.5.8.0
plygeneration: /usr/lib/libvtkIO.so.5.8.0
plygeneration: /usr/lib/libvtkFiltering.so.5.8.0
plygeneration: /usr/lib/libvtkCommon.so.5.8.0
plygeneration: /usr/lib/libvtksys.so.5.8.0
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_nonfree.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
plygeneration: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
plygeneration: CMakeFiles/plygeneration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable plygeneration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plygeneration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plygeneration.dir/build: plygeneration
.PHONY : CMakeFiles/plygeneration.dir/build

CMakeFiles/plygeneration.dir/requires: CMakeFiles/plygeneration.dir/plyfileGeneration.cpp.o.requires
CMakeFiles/plygeneration.dir/requires: CMakeFiles/plygeneration.dir/rply.c.o.requires
.PHONY : CMakeFiles/plygeneration.dir/requires

CMakeFiles/plygeneration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plygeneration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plygeneration.dir/clean

CMakeFiles/plygeneration.dir/depend:
	cd /home/ros/Downloads/exp14/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/Downloads/exp14 /home/ros/Downloads/exp14 /home/ros/Downloads/exp14/build /home/ros/Downloads/exp14/build /home/ros/Downloads/exp14/build/CMakeFiles/plygeneration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plygeneration.dir/depend

