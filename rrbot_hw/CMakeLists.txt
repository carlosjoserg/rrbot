cmake_minimum_required(VERSION 2.8.3)
project(rrbot_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  gazebo_ros
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    gazebo_ros
  LIBRARIES 
    rrbot_hw
    default_rrbot_hw_sim
    rrbot_hw_sim_plugin
  DEPENDS
    gazebo
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Hardware Interface Sim
add_library(default_rrbot_hw_sim src/default_rrbot_hw_sim.cpp)
target_link_libraries(default_rrbot_hw_sim
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Gazebo plugin for Hardware Interface Sim
add_library(rrbot_hw_sim_plugin src/rrbot_hw_sim_plugin.cpp)
add_dependencies(rrbot_hw_sim_plugin
  default_rrbot_hw_sim
)
target_link_libraries(rrbot_hw_sim_plugin
  default_rrbot_hw_sim
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Hardware Interface Real
add_library(rrbot_hw_real src/rrbot_hw_real.cpp)
target_link_libraries(rrbot_hw_real
  ${catkin_LIBRARIES}
)

# ROS Node for Hardware Interface Real
add_executable(rrbot_hw_real_node src/rrbot_hw_real_node.cpp)
add_dependencies(rrbot_hw_real_node
  rrbot_hw_real
)
target_link_libraries(rrbot_hw_real_node
  rrbot_hw_real
  ${catkin_LIBRARIES}
)
