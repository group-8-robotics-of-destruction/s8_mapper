set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_mapper)

find_package(catkin REQUIRED COMPONENTS roscpp s8_common_node s8_ir_sensors s8_pose s8_msgs message_generation s8_turner s8_motor_controller geometry_msgs)

add_service_files(
  FILES
  PlaceNode.srv
)

add_action_files(
  DIRECTORY action
  FILES Navigate.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_common_node s8_ir_sensors s8_pose s8_msgs message_runtime s8_turner s8_motor_controller geometry_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_mapper_node src/mapper_node.cpp)
target_link_libraries(s8_mapper_node ${catkin_LIBRARIES})
