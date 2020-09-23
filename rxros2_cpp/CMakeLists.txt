cmake_minimum_required(VERSION 3.5)
project(rxros2_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################################################
# TODO: put a proper vendor package together

include(ExternalProject)
ExternalProject_Add(rxcpp-6c283f36
  SOURCE_DIR rxcpp_src
  URL https://github.com/ReactiveX/RxCpp/archive/v4.1.0.tar.gz
  URL_MD5 6c283f36ce251f45146f7099aa9ef19a
  # arbitrary
  TIMEOUT 60
  # we copy the headers out of the extracted source directly, so
  # no need to configure, build or install anything.
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

# NOTE: this places a copy of the RxCpp headers in the public include path
# of this package. Users who depend on rxros2_cpp probably want access to
# those headers, but this is not nice, and we should put a proper vendor
# package together (or find some other way to package RxCpp for ROS 2).
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rxcpp_src/Rx/v2/src/rxcpp
  DESTINATION include
)
################################################################################


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
