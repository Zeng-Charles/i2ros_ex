cmake_minimum_required(VERSION 3.0.2)
project(customers)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(customers src/clients_e2.cpp)
target_link_libraries(customers
  ${catkin_LIBRARIES}
)

add_executable(customers_client src/Clients_e3.cpp)
target_link_libraries(customers_client
  ${catkin_LIBRARIES}
)