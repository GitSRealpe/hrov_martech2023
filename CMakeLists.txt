cmake_minimum_required(VERSION 3.0.2)
project(hrov_martech2023)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    nav_msgs
    tf
    tf2_ros
    laser_geometry
    sensor_msgs
    message_filters
    filters
    interactive_markers
    visualization_msgs

    octomap_msgs
    octomap_ros
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation)

add_message_files(
  FILES
  PointArray.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)

catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

include_directories(SYSTEM ${INCLUDES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hrov_martech2023_node.cpp)
add_executable(${PROJECT_NAME}_odom2tf scripts/odom2tf.cpp)
add_executable(${PROJECT_NAME}_laser2ptcloud scripts/laser2ptcloud.cpp)
add_executable(${PROJECT_NAME}_octo_proc scripts/octo_proc.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_odom2tf PROPERTIES OUTPUT_NAME odom2tf PREFIX "")
set_target_properties(${PROJECT_NAME}_laser2ptcloud PROPERTIES OUTPUT_NAME laser2ptcloud PREFIX "")
set_target_properties(${PROJECT_NAME}_octo_proc PROPERTIES OUTPUT_NAME octo_proc PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME}_laser2ptcloud ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_odom2tf ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_octo_proc ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/joint_traj_ctr_pub.py
  scripts/lsrptcld.py
  scripts/clustering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
