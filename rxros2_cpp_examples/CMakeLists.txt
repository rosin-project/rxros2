cmake_minimum_required(VERSION 3.5)
project(rxros2_cpp_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rxros2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(listener_new_style src/listener_new_style.cpp)
ament_target_dependencies(listener_new_style rxros2_cpp std_msgs)

add_executable(listener_old_style src/listener_old_style.cpp)
ament_target_dependencies(listener_old_style rxros2_cpp std_msgs)

add_executable(talker_new_style src/talker_new_style.cpp)
ament_target_dependencies(talker_new_style rxros2_cpp std_msgs)

add_executable(talker_old_style src/talker_old_style.cpp)
ament_target_dependencies(talker_old_style rxros2_cpp std_msgs)

install(TARGETS
  listener_new_style
  listener_old_style
  talker_new_style
  talker_old_style
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
