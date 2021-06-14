cmake_minimum_required(VERSION 3.5)
project(kr_ros2_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

link_directories(lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(EXTRALIB_BIN
        kr_msg_ros2__python
        kr_msg_ros2__rosidl_generator_c
        kr_msg_ros2__rosidl_typesupport_c
        kr_msg_ros2__rosidl_typesupport_cpp
        kr_msg_ros2__rosidl_typesupport_fastrtps_c
        kr_msg_ros2__rosidl_typesupport_fastrtps_cpp
        kr_msg_ros2__rosidl_typesupport_introspection_c
        kr_msg_ros2__rosidl_typesupport_introspection_cpp
        )


add_executable(kr_ros2_test src/kr_ros2_test.cpp)

target_link_libraries(kr_ros2_test ${EXTRALIB_BIN})

ament_target_dependencies(kr_ros2_test rclcpp std_msgs)

install(TARGETS
        kr_ros2_test
        DESTINATION lib/${PROJECT_NAME})

ament_package()