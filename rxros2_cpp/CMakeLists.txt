cmake_minimum_required(VERSION 3.5)
project(rxros2_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# TODO: hasn't been released yet, so we can't release like this
# find_package(rxcpp_vendor REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
 INTERFACE
 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
 "$<INSTALL_INTERFACE:include>")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_14)

# link our dependencies
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp rclcpp_action)

# these should not be necessary, as we use ament_export_targets?
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)

# notify consuming Ament packages of (transitive) dependencies
if(COMMAND ament_export_targets)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
else()
  ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
