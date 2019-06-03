cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_2Dmap_plugin)

SET(CMAKE_BUILD_TYPE Debug)

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
  message_generation
)

add_definitions(-std=c++11)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_service_files(
   FILES
   SaveMap.srv
   GenerateMap.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_2Dmap_plugin SHARED src/gazebo_2Dmap_plugin.cpp)
target_link_libraries(gazebo_2Dmap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_2Dmap_plugin ${PROJECT_NAME}_gencpp)

add_executable(save_map src/map_saver.cpp )
target_link_libraries(save_map ${catkin_LIBRARIES})
add_dependencies(save_map ${PROJECT_NAME}_gencpp)


install(TARGETS gazebo_2Dmap_plugin
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
