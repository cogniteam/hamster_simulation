cmake_minimum_required(VERSION 2.8.3)
project(hamster_teleop)

find_package(
  catkin REQUIRED COMPONENTS
  ackermann_msgs
  roscpp
  sensor_msgs
  geometry_msgs
)
find_package(Boost REQUIRED COMPONENTS)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hamster_teleop_node
	src/joystick_teleop_node.cpp)

target_link_libraries(hamster_teleop_node
  ${catkin_LIBRARIES}
)
