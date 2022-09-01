cmake_minimum_required(VERSION 2.8.3)
project(lidar_utils)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  cdf_msgs
  tf
  sender
)

catkin_package()

include_directories(
# include
    pigpiod_if2
    ${catkin_INCLUDE_DIRS}
)

#add_executable(dev_lidar src/dev.cpp)
add_executable(detect_obstacle src/detect_obstacle.cpp)

target_link_libraries(detect_obstacle ${catkin_LIBRARIES} ${roscpp_LIBRARIES} pigpiod_if2 pthread)
