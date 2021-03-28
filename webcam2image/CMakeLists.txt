# cmake_minimum_required(VERSION 3.5)
# project(webcam2image)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

# # Add executables
# add_executable(webcam2image src/webcam2image.cpp)
# ament_target_dependencies(webcam2image rclcpp std_msgs sensor_msgs)

# install(TARGETS
#   webcam2image
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(webcam2image)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(webcam2image src/webcam2image.cpp)
ament_target_dependencies(webcam2image rclcpp std_msgs sensor_msgs OpenCV)

install(TARGETS
  webcam2image
  DESTINATION lib/${PROJECT_NAME})

ament_package()