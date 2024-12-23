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
  tf
  pcl_ros
  message_generation
  message_runtime
)

add_message_files(
  FILES
  GridMapUpdateMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  grid_map_msgs
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

# add_executable(gridmap_node src/gridmap_node.cpp)
# target_link_libraries(gridmap_node ${catkin_LIBRARIES})

add_executable(multi_layer_map_node src/multi_layer_map_node.cpp)
target_link_libraries(multi_layer_map_node ${catkin_LIBRARIES})

add_executable(cloud_filter_node src/cloud_filter_node.cpp)
target_link_libraries(cloud_filter_node ${catkin_LIBRARIES})

add_executable(elevation_node src/elevation_node.cpp)
target_link_libraries(elevation_node ${catkin_LIBRARIES})

add_executable(surface_orientation_node src/surface_orientation_node.cpp)
target_link_libraries(surface_orientation_node ${catkin_LIBRARIES})

add_executable(obstacles_node src/obstacles_node.cpp)
target_link_libraries(obstacles_node ${catkin_LIBRARIES})

add_executable(link_states_to_odom_node src/link_states_to_odom_node.cpp)
target_link_libraries(link_states_to_odom_node ${catkin_LIBRARIES})



