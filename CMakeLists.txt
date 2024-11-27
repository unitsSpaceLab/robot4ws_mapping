cmake_minimum_required(VERSION 3.0.2)
project(robot4ws_mapping)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_loader
  grid_map_msgs
  grid_map_ros
  grid_map_rviz_plugin
  rosbag
  std_msgs
  tf2_ros
  sensor_msgs
  costmap_2d
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot4ws_mapping
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Mark other files for installation
install(
  DIRECTORY config data doc launch rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(gridmap_node src/gridmap_node.cpp)
target_link_libraries(gridmap_node ${catkin_LIBRARIES})



