cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  sensor_msgs
  morai_msgs
  geodesy
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs morai_msgs geodesy geographic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit src/pure_pursuit.cpp src/pure_pursuit_function.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})

add_executable(path_recorder src/path_recorder.cpp src/pure_pursuit_function.cpp)
target_link_libraries(path_recorder ${catkin_LIBRARIES})

