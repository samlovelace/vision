cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Explicitly look for OpenCV in /usr/local
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/CameraFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/CameraHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/DepthEstimatorFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/EngineFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/InferenceHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/MidasModel.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ModelFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/MonocularDepthEstimator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ObjectDetectionHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/OpenCvEngine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PoseEstimationHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RateController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/UsbCamera.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/Vision.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/YoloModel.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/include 
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
)
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

ament_package()
