cmake_minimum_required(VERSION 3.0.2)
project(height_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  grid_map_ros
  grid_map_msgs
  grid_map_visualization
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES height_map
 CATKIN_DEPENDS roscpp pcl_conversions pcl_ros sensor_msgs visualization_msgs grid_map_ros grid_map_msgs grid_map_visualization
 DEPENDS PCL
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/generate_height_map.cpp
  include/${PROJECT_NAME}/mapping_height_map.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(height_map_test_node include/${PROJECT_NAME}/test.cpp)
target_link_libraries(height_map_test_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(height_map_node src/height_map_node.cpp)
target_link_libraries(height_map_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(local_plane_map_node src/local_plane_map_node.cpp)
target_link_libraries(local_height_map_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable(local_height_map_node src/local_height_map_node.cpp)
target_link_libraries(local_height_map_nodes
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} height_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} local_plane_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} local_height_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} height_map_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


