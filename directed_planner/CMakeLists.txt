cmake_minimum_required(VERSION 2.8.3)
project(directed_planner)

find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    message_generation
    directed_msgs
    directed_layer
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES directed_planner
  CATKIN_DEPENDS
    base_local_planner
    dynamic_reconfigure
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
    message_runtime
    directed_msgs
    directed_layer
)

add_library(directed_planner src/directed_planner.cpp src/dijkstra.cpp src/astar.cpp)

add_dependencies(directed_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(directed_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS directed_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
