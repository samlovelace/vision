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

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/EngineFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ModelFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ModelHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/OpenCvEngine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/YoloModel.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/include 
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
)
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp 
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

ament_package()
