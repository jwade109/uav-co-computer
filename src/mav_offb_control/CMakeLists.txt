cmake_minimum_required(VERSION 3.0.2)
project(mav_offb_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
)

# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(mav_offb_control_node src/offb_node.cpp)
add_executable(mavros_fly_mission_node src/mavros_fly_mission_node.cpp)
add_executable(read_node src/testread.cpp)
add_executable(write_node src/testwrite.cpp)

target_link_libraries(mav_offb_control_node
  ${catkin_LIBRARIES}
)

target_link_libraries(mavros_fly_mission_node
  ${catkin_LIBRARIES}
)

target_link_libraries(read_node
  ${catkin_LIBRARIES}
)

target_link_libraries(write_node
  ${catkin_LIBRARIES}
)
