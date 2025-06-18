cmake_minimum_required(VERSION 3.0.2)
project(superos)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(superos_node src/SupeROS_e1.cpp)

target_link_libraries(superos_node
  ${catkin_LIBRARIES}
)

