cmake_minimum_required(VERSION 3.8)
project(iris_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plog REQUIRED)

# header-only package - no .cpp sources, just an INTERFACE target so
# consumers get the include path and transitive deps via find_package()
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  plog::plog
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp plog)

ament_package()
