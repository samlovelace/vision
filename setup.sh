#!/bin/bash

WS=$1
source "$WS/src/robot_idl/scripts/common.sh" "$@"

LIB_DIR="/opt"
OPENCV_VERSION=4.7.0
LIB_INSTALL_DIR="/usr/local"

# Subtract 2 from total cores
CORES=$(( $(nproc) - 4 ))

# Ensure at least 1 core is used
if [ "$CORES" -lt 1 ]; then
  CORES=1
fi

# List of required packages
DEPENDENCIES=(
    build-essential
    cmake
    git
    curl
    libyaml-cpp-dev
    libssl-dev # start of deps for librealsense/opencv 
    libx11-dev
    libxrandr-dev
    libxinerama-dev
    libxcursor-dev
    libxi-dev
    libgl1-mesa-dev
    libglu1-mesa-dev
    libudev-dev
    libusb-1.0-0-dev
    libeigen3-dev # start of deps for pcl 
    libflann-dev 
    libboost-all-dev 
    libqhull-dev 
    libpng-dev 
    libjpeg-dev 
    libtiff-dev 
    libpcap-dev
)

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (e.g., sudo ./setup.sh)"
    exit 1
fi

# treat ros2 install special 
check_and_install "ros-humble-desktop" "install_ros"

# Loop through and install each
for pkg in "${DEPENDENCIES[@]}"; do
    check_and_install "$pkg"
done

# Ensure the directory exists
mkdir -p "$LIB_DIR"
mkdir -p "$WS"/src

# Assumes robot_idl was cloned first and runs this install script
#git clone https://github.com/samlovelace/robot_idl.git "$WS/src/robot_idl"

 #librealsense 
cd "$LIB_DIR"
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense 
mkdir build && cd build 
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DFORCE_RSUSB_BACKEND=TRUE .. && make -j"$CORES"
make install 

# # opencv
cd "$LIB_DIR"
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib 
git checkout "$OPENCV_VERSION"

cd "$LIB_DIR"
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout "$OPENCV_VERSION" 

mkdir build && cd build 
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX="$LIB_INSTALL_DIR" \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF .. 
make -j"$CORES" && make install && ldconfig

#pcl 
cd "$LIB_DIR"
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout pcl-1.15.0
mkdir build && cd build 
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_tools=OFF \
  -DBUILD_examples=OFF \
  -DWITH_VTK=ON

make -j"$CORES" && make install && ldconfig

cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_idl vision 





