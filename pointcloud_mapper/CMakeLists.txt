cmake_minimum_required(VERSION 3.5)
project(pointcloud_mapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL COMPONENTS REQUIRED)

add_executable(pointcloud_mapper
        src/pointcloud_mapper.cpp
        )
ament_target_dependencies(pointcloud_mapper
        OpenCV
        PCL
        rclcpp
        sensor_msgs
        cv_bridge
        geometry_msgs
        tf2_geometry_msgs
        image_transport
        message_filters)

#FIND_LIBRARY(PCL_LIBRARIES PCL HINTS /usr/in)
target_link_libraries(pointcloud_mapper ${PCL_LIBRARIES})

install(TARGETS pointcloud_mapper DESTINATION lib/${PROJECT_NAME})
ament_package()
