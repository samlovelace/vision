cmake_minimum_required(VERSION 3.8)
project(iris_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(iris_msgs REQUIRED)
find_package(iris_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(plog REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)

# Explicitly look for OpenCV in /usr/local, matching iris_core's convention
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/CameraDriver.cpp
  src/CameraFactory.cpp
  src/RateController.cpp
  src/RosCamera.cpp
  src/UsbCamera.cpp
  src/RealSenseCamera_d435i.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/humble/include/cv_bridge
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  iris_msgs
  iris_common
  message_filters
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${OpenCV_LIBS}
  realsense2
  plog::plog
)

# needed for some reason? (matches iris_core's build - ABI mismatch workaround)
target_compile_definitions(${PROJECT_NAME} PRIVATE _GLIBCXX_USE_CXX11_ABI=1)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
