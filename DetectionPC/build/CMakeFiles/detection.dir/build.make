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
CMAKE_SOURCE_DIR = /home/dan/GitHub/ImageProcessing/DetectionPC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dan/GitHub/ImageProcessing/DetectionPC/build

# Include any dependencies generated for this target.
include CMakeFiles/detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/detection.dir/flags.make

CMakeFiles/detection.dir/src/main.cpp.o: CMakeFiles/detection.dir/flags.make
CMakeFiles/detection.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dan/GitHub/ImageProcessing/DetectionPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/detection.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/detection.dir/src/main.cpp.o -c /home/dan/GitHub/ImageProcessing/DetectionPC/src/main.cpp

CMakeFiles/detection.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/detection.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dan/GitHub/ImageProcessing/DetectionPC/src/main.cpp > CMakeFiles/detection.dir/src/main.cpp.i

CMakeFiles/detection.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/detection.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dan/GitHub/ImageProcessing/DetectionPC/src/main.cpp -o CMakeFiles/detection.dir/src/main.cpp.s

# Object files for target detection
detection_OBJECTS = \
"CMakeFiles/detection.dir/src/main.cpp.o"

# External object files for target detection
detection_EXTERNAL_OBJECTS =

detection: CMakeFiles/detection.dir/src/main.cpp.o
detection: CMakeFiles/detection.dir/build.make
detection: /installation/OpenCV-/lib/libopencv_highgui.so
detection: /installation/OpenCV-/lib/libopencv_core.so
detection: /installation/OpenCV-/lib/libopencv_imgcodecs.so
detection: /installation/OpenCV-/lib/libopencv_imgproc.so
detection: /installation/OpenCV-/lib/libopencv_photo.so
detection: /installation/OpenCV-/lib/libopencv_calib3d.so
detection: CMakeFiles/detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dan/GitHub/ImageProcessing/DetectionPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/detection.dir/build: detection

.PHONY : CMakeFiles/detection.dir/build

CMakeFiles/detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/detection.dir/clean

CMakeFiles/detection.dir/depend:
	cd /home/dan/GitHub/ImageProcessing/DetectionPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dan/GitHub/ImageProcessing/DetectionPC /home/dan/GitHub/ImageProcessing/DetectionPC /home/dan/GitHub/ImageProcessing/DetectionPC/build /home/dan/GitHub/ImageProcessing/DetectionPC/build /home/dan/GitHub/ImageProcessing/DetectionPC/build/CMakeFiles/detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/detection.dir/depend

