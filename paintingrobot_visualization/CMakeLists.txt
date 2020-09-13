cmake_minimum_required(VERSION 2.8.3)
project(paintingrobot_visualization)


find_package(catkin REQUIRED COMPONENTS
  angles
  pcl_conversions
  pcl_ros
  roscpp
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  roscpp
  rospy
  sensor_msgs 
  tf

)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  LIBRARIES
    tf2_geometry_msgs
    geometry_msgs
  DEPENDS
      EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(paintingrobotscene_visualization src/paintingrobotscene_visualization.cpp)
target_link_libraries(paintingrobotscene_visualization ${catkin_LIBRARIES} ${PCL_LIBS})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
