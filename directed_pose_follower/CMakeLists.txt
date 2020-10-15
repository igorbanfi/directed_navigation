cmake_minimum_required(VERSION 2.8.3)
project(directed_pose_follower)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            pose_follower
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
            directed_msgs
            directed_layer
            dwa_local_planner
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_local_planner base_local_planner pose_follower
    CATKIN_DEPENDS
        base_local_planner
        pose_follower
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
        directed_msgs
        directed_layer
)

add_library(directed_pose_follower src/directed_pose_follower.cpp)
add_dependencies(directed_pose_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(directed_pose_follower ${catkin_LIBRARIES})

install(TARGETS directed_pose_follower
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
