cmake_minimum_required(VERSION 3.0.2)
project(mid_project)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  urdf
  xacro
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS
  "${GAZEBO_CXX_FLAGS}"
)

## Declare a C++ library
add_library(moving_plugin_bottom SHARED src/moving_plugin_bottom.cc)
target_link_libraries(moving_plugin_bottom ${GAZEBO_LIBRARIES})

add_library(moving_plugin_top SHARED src/moving_plugin_top.cc)
target_link_libraries(moving_plugin_top ${GAZEBO_LIBRARIES})

add_library(moving_plugin_left SHARED src/moving_plugin_left.cc)
target_link_libraries(moving_plugin_left ${GAZEBO_LIBRARIES})

add_library(moving_plugin_right SHARED src/moving_plugin_right.cc)
target_link_libraries(moving_plugin_right ${GAZEBO_LIBRARIES})