cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nora_idl REQUIRED)
find_package(vision_idl REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PCL_DIR "/usr/local/pcl-1.15/lib/cmake/pcl")
find_package(PCL REQUIRED)

# Explicitly look for OpenCV in /usr/local
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

add_subdirectory(src/camera)
add_subdirectory(src/general)
add_subdirectory(src/inference)
add_subdirectory(src/model)
add_subdirectory(src/pose)

add_executable(${PROJECT_NAME} src/main.cpp)

set(SOURCES 
  ${CAMERA_SOURCES}
  ${GENERAL_SOURCES}
  ${INFERENCE_SOURCES}
  ${MODEL_SOURCES}
  ${POSE_SOURCES}
)

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})

include_directories(${PROJECT_NAME}
  src/camera
  src/general
  src/inference
  src/model
  src/pose
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nora_idl
  vision_idl
)
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

ament_package()
