# ImageProcessing

cd build
cmake --build . --config Release

To rename all calibration files:
ls -v | cat -n | while read n f; do mv -n "$f" "$n.jpg"; done 


# _____________________________________________________________
# Install CMake from source: 
# Check latest version from (https://cmake.org/download/)

sudo apt remove --purge cmake

sudo apt install build-essential libssl-dev
wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
tar -zxvf cmake-3.20.2.tar.gz
cd cmake-3.20.2
./bootstrap
make 
sudo make install 

# _____________________________________________________________
# Install OpenCV on Linux:

# Clean build directories
rm -rf opencv/build
rm -rf opencv_contrib/build

# Create directory for installation
mkdir installation

# Update packages
sudo apt -y update
sudo apt -y upgrade

# Install OS Libraries
sudo apt -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt -y install libgtk2.0-dev libtbb-dev qt5-default
sudo apt -y install libatlas-base-dev
sudo apt -y install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt -y install libvorbis-dev libxvidcore-dev
sudo apt -y install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt -y install libavresample-dev
sudo apt -y install x264 v4l-utils

# Optional dependencies
sudo apt -y install libprotobuf-dev protobuf-compiler
sudo apt -y install libgoogle-glog-dev libgflags-dev
sudo apt -y install libgphoto2-dev libeigen3-dev libhdf5-dev doxygen

# Download opencv and opencv_contrib
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout $cvVersion

# Compile and install OpenCV with contrib modules
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-"$cvVersion" \
-D BUILD_LIST=dnn,core,calib3d,imgproc,imgcodecs,highgui,photo \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_opencv_python=OFF \
-D WITH_TBB=OFF \
-D WITH_V4L=OFF \
-D WITH_QT=OFF \
-D WITH_OPENGL=OFF \
-D BUILD_EXAMPLES=OFF \
-D BUILD_opencv_java=OFF \
-D BUILD_opencv_python=OFF \
-D BUILD_opencv_ts=OFF \
-D CMAKE_CXX_FLAGS="-std=c++17" ..
 	
make -j4
make install

# If running via ssh to run in background
nohup make -j4 & exit

tail -f nohup.out