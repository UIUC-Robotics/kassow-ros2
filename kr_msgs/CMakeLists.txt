cmake_minimum_required(VERSION 3.5)
project(kr_msgs)

# Get ROS_DISTRO from the environment
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable is not defined. Please source your ROS 2 setup.bash.")
endif()

set(ROS_DISTRO "$ENV{ROS_DISTRO}")
message(STATUS "Detected ROS 2 distribution: ${ROS_DISTRO}")

# Include version-specific CMake logic
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${ROS_DISTRO}.cmake")
  include(cmake/${ROS_DISTRO}.cmake)
else()
  message(WARNING "No cmake/${ROS_DISTRO}.cmake file found. Falling back to default.cmake.")
  include(cmake/default.cmake)
endif()

