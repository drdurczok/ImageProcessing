cmake_minimum_required(VERSION 3.1)
project(benchmark)
# Enable C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# provide library headers
include_directories(/installation/OpenCV-/include/opencv4/)
add_executable(benchmark ../src/main.cpp)

# link OpenCV libraries
target_link_libraries( benchmark /installation/OpenCV-/lib/libopencv_highgui.so /installation/OpenCV-/lib/libopencv_core.so /installation/OpenCV-/lib/libopencv_imgcodecs.so /installation/OpenCV-/lib/libopencv_imgproc.so)

# display opencv libs found
MESSAGE("OpenCV_LIBS: " /installation/OpenCV-/lib/ )
