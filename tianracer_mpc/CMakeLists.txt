cmake_minimum_required(VERSION 3.0.2)
project(tianracer_mpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_core
  tf2_ros
  costmap_2d
  pluginlib
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  LIBRARIES mpc_local_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_core tf2_ros costmap_2d pluginlib ackermann_msgs geometry_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(mpc_local_planner src/mpc_local_planner.cpp)
target_link_libraries(mpc_local_planner ${catkin_LIBRARIES})

install(TARGETS mpc_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/model_predictive_speed_and_steer_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
