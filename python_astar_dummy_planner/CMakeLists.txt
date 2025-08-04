cmake_minimum_required(VERSION 3.0.2)
project(python_astar_dummy_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_core
  pluginlib
  costmap_2d
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dummy_global_planner
  CATKIN_DEPENDS roscpp nav_core pluginlib costmap_2d geometry_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dummy_global_planner
  src/dummy_global_planner.cpp
)

target_link_libraries(dummy_global_planner
  ${catkin_LIBRARIES}
)

install(TARGETS dummy_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  dummy_global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/simple_astar_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
