cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_2Dmap_plugin)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
  joint_state_publisher
  kdl_parser
  robot_state_publisher
  roscpp
  rospy
  tf
  xacro
)

add_definitions(-std=c++11)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_2Dmap_plugin SHARED src/gazebo_2Dmap_plugin.cpp)
target_link_libraries(gazebo_2Dmap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_2Dmap_plugin
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
