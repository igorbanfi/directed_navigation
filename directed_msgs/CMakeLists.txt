cmake_minimum_required(VERSION 3.0.2)
project(directed_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation std_msgs actionlib_msgs nav_msgs)

add_message_files(
   FILES
   DirectedMap.msg
   DirectedMapUpdate.msg
)

add_service_files(
  FILES
  GetDirectedMap.srv
)

add_action_files(
  FILES
  GetDirectedMap.action
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs actionlib_msgs)
