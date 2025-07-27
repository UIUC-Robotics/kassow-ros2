cmake_minimum_required(VERSION 3.5)
project(kr_example_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined. Source ROS" )
endif()
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  message(STATUS "Build for ROS2 Foxy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DFOXY")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "dashing")
  message(STATUS "Build for ROS2 Dashing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDASHING")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Build for ROS2 Humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHUMBLE")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Build for ROS2 Iron")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIRON")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  message(STATUS "Build for ROS2 Rolling")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DROLLING")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Build for ROS2 Jazzy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DJAZZY")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "kilted")
  message(STATUS "Build for ROS2 Kilted")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DKILTED")
  set(SOURCES "${SOURCES}" src/jog_linear.cpp)
else()
  message(FATAL_ERROR "Unsupported ROS Distribution: " "$ENV{ROS_DISTRO}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kr_msgs REQUIRED)

add_executable(robot_state src/robot_state.cpp)
add_executable(follow_joint src/follow_joint.cpp)
add_executable(move_joint src/move_joint.cpp)
add_executable(jog_linear src/jog_linear.cpp)
add_executable(self_motion src/self_motion.cpp)

ament_target_dependencies(robot_state rclcpp std_msgs kr_msgs)
ament_target_dependencies(follow_joint rclcpp std_msgs kr_msgs)
ament_target_dependencies(move_joint rclcpp std_msgs kr_msgs)
ament_target_dependencies(jog_linear rclcpp std_msgs kr_msgs)
ament_target_dependencies(self_motion rclcpp std_msgs kr_msgs)

install(TARGETS
  robot_state
  follow_joint
  move_joint
  jog_linear
  self_motion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
