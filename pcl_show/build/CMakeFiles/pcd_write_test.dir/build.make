# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pretend/code/pcl_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pretend/code/pcl_test/build

# Include any dependencies generated for this target.
include CMakeFiles/pcd_write_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pcd_write_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pcd_write_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcd_write_test.dir/flags.make

CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o: CMakeFiles/pcd_write_test.dir/flags.make
CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o: ../pcd_write.cpp
CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o: CMakeFiles/pcd_write_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pretend/code/pcl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o -MF CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o.d -o CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o -c /home/pretend/code/pcl_test/pcd_write.cpp

CMakeFiles/pcd_write_test.dir/pcd_write.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcd_write_test.dir/pcd_write.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pretend/code/pcl_test/pcd_write.cpp > CMakeFiles/pcd_write_test.dir/pcd_write.cpp.i

CMakeFiles/pcd_write_test.dir/pcd_write.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcd_write_test.dir/pcd_write.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pretend/code/pcl_test/pcd_write.cpp -o CMakeFiles/pcd_write_test.dir/pcd_write.cpp.s

# Object files for target pcd_write_test
pcd_write_test_OBJECTS = \
"CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o"

# External object files for target pcd_write_test
pcd_write_test_EXTERNAL_OBJECTS =

pcd_write_test: CMakeFiles/pcd_write_test.dir/pcd_write.cpp.o
pcd_write_test: CMakeFiles/pcd_write_test.dir/build.make
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_people.so
pcd_write_test: /usr/lib/libOpenNI.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_features.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_search.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_io.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpng.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libz.so
pcd_write_test: /usr/lib/libOpenNI.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libfreetype.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libGLEW.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libX11.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.3
pcd_write_test: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
pcd_write_test: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
pcd_write_test: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libtbb.so.12.5
pcd_write_test: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libpcl_common.so
pcd_write_test: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.74.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
pcd_write_test: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
pcd_write_test: CMakeFiles/pcd_write_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pretend/code/pcl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pcd_write_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcd_write_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcd_write_test.dir/build: pcd_write_test
.PHONY : CMakeFiles/pcd_write_test.dir/build

CMakeFiles/pcd_write_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcd_write_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcd_write_test.dir/clean

CMakeFiles/pcd_write_test.dir/depend:
	cd /home/pretend/code/pcl_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pretend/code/pcl_test /home/pretend/code/pcl_test /home/pretend/code/pcl_test/build /home/pretend/code/pcl_test/build /home/pretend/code/pcl_test/build/CMakeFiles/pcd_write_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcd_write_test.dir/depend
