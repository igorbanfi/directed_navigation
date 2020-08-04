cmake_minimum_required(VERSION 3.0.2)
project(directed_map_server)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            directed_msgs
            tf2
        )

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        directed_map_server_image_loader
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        directed_msgs
        tf2
)

include_directories(
    include
    ${BULLET_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(directed_map_server_image_loader src/image_loader.cpp)
add_dependencies(directed_map_server_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(directed_map_server_image_loader
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)

add_executable(directed_map_server src/main.cpp)
add_dependencies(directed_map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(directed_map_server
    directed_map_server_image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)


install(TARGETS directed_map_server_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
