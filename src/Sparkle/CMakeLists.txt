cmake_minimum_required(VERSION 3.0.2)
project(Sparkle)

set(CMAKE_CXX_STANDARD 14) # 추가 


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(lidar_convert src/lidar/convert_lidar.cpp)
# target_link_libraries(lidar_convert ${catkin_LIBRARIES})
