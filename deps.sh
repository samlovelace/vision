#!/bin/bash

LIB_DIR="/opt"
OPENCV_VERSION=4.7.0
LIB_INSTALL_DIR="/usr/local"

# Subtract 2 from total cores
CORES=$(( $(nproc) - 2 ))

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

install_librealsense()
{
	REPO_NAME="librealsense"

	if [ -d "$LIB_DIR/$REPO_NAME/.git" ]; then
    	echo "$REPO_NAME already cloned at $LIB_DIR. Skipping clone and assuming proper build..."
		return
	fi

	#librealsense 
	cd "$LIB_DIR"
	git clone https://github.com/IntelRealSense/librealsense.git
	cd "$REPO_NAME" 
	mkdir build && cd build 
	cmake -DCMAKE_INSTALL_PREFIX="$LIB_INSTALL_DIR" -DFORCE_RSUSB_BACKEND=TRUE .. && make -j"$CORES"
	$SUDO make install 
}

install_opencv()
{
	REPO_NAME="opencv_contrib"

	if [ -d "$LIB_DIR/$REPO_NAME/.git" ]; then
    	echo "$REPO_NAME already cloned at $LIB_DIR. Skipping clone and assuming proper build..."
		return
	fi

	# opencv
	cd "$LIB_DIR"
	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv_contrib 
	git checkout "$OPENCV_VERSION"

	REPO_NAME="opencv"

	if [ -d "$LIB_DIR/$REPO_NAME/.git" ]; then
    	echo "$REPO_NAME already cloned at $LIB_DIR. Skipping clone and assuming proper build..."
		return
	fi

	cd "$LIB_DIR"
	git clone https://github.com/opencv/opencv.git
	cd opencv 
	git checkout "$OPENCV_VERSION" 

	mkdir build && cd build 
	cmake -D CMAKE_BUILD_TYPE=Release \
		  -D CMAKE_INSTALL_PREFIX="$LIB_INSTALL_DIR" \
		  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
		  -D BUILD_EXAMPLES=OFF .. 
	make -j"$CORES" && $SUDO make install && ldconfig
}

install_pcl()
{
	REPO_NAME="pcl"

	if [ -d "$LIB_DIR/$REPO_NAME/.git" ]; then
    	echo "$REPO_NAME already cloned at $LIB_DIR. Skipping clone and assuming proper build..."
		return
	fi

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

	make -j"$CORES" && $SUDO make install && ldconfig

}

# define the set of functions to be called by robot_idl setup script 
function run_custom_build_steps() {
    install_librealsense
    install_opencv
    install_pcl
}


