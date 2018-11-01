cmake_minimum_required(VERSION 2.8.3)
project(simple_navigation_goals)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
angles
    dynamic_reconfigure
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_geometry_msgs
    tf2_ros
	global_planner
)
generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

catkin_package(
INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    dynamic_reconfigure
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
	global_planner
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
add_dependencies(simple_navigation_goals ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_navigation_goals
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} simple_navigation_goals
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
