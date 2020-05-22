cmake_minimum_required(VERSION 2.8.3)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_navi_node src/navi_node.cpp)

target_link_libraries(${PROJECT_NAME}_navi_node ${catkin_LIBRARIES})



