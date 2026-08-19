cmake_minimum_required(VERSION 3.5)
project(modelo_omnidireccional)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects

include_directories(include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ executable
add_executable(youbot_odometry_node src/youbot_odometry_node.cpp src/youbot_odometry.cpp)
ament_target_dependencies(youbot_odometry_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  robmovil_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(logger src/logger_node.cpp)
ament_target_dependencies(logger
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  robmovil_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS youbot_odometry_node logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ament package macro
ament_package()
