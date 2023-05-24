cmake_minimum_required(VERSION 3.0.2)
project(hrov_martech2023)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# set(ompl_DIR /home/srealper/theLibs/ompl/build/Release/)
# set(OMPL_LIBRARIES /home/srealper/theLibs/ompl/build/Release/lib/libompl.so)
find_package(ompl REQUIRED)
message("ompl print")
message("1 ${OMPL_FOUND}")
message("2 ${OMPL_INCLUDE_DIRS}")
message("3 ${OMPL_LIBRARY_DIRS}")
message("4 ${OMPL_LIBRARY_DIR}")
message("5 ${OMPL_LIBRARIES}")
message("${OMPL_VERSION}")
find_package(fcl REQUIRED)
message("fcl prints")
# message("1 ${fcl_FOUND}")
# message("2 ${FCL_INCLUDE_DIRS}")
# message("3 ${FCL_LIBRARY_DIRS}")
# message("4 ${FCL_LIBRARIES}")
# message("5 ${FCL_VERSION}")


list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    nav_msgs
    tf
    tf2_ros
    laser_geometry
    message_filters
    filters
    interactive_markers
    visualization_msgs
    octomap_msgs
    octomap_ros
    cola2_msgs

    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    shape_msgs
    octomap_msgs
    trajectory_msgs
    object_recognition_msgs
    rosbag
    urdf
    srdfdom
    pluginlib
    geometric_shapes
    tf2_ros
    xmlrpcpp
  
    # moveit_planners_ompl
    rviz_visual_tools

  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation)

add_message_files(
  FILES
  PointArray.msg
)

add_service_files(
  FILES
  PlanGoal.srv
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

###########s
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  
  )

include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
    ${OMPL_LIBRARIES}
    ${FCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hrov_martech2023_node.cpp)
add_executable(${PROJECT_NAME}_laser2ptcloud scripts/utils/laser2ptcloud.cpp)
add_executable(${PROJECT_NAME}_octo_proc1 scripts/octo_proc1.cpp)
add_executable(${PROJECT_NAME}_octo_proc2 scripts/octo_proc2.cpp)
add_executable(${PROJECT_NAME}_planning2 scripts/planning2.cpp)
add_executable(${PROJECT_NAME}_planning2_srv scripts/planning2_srv.cpp)
add_executable(${PROJECT_NAME}_planning2dubs scripts/planning2dubs.cpp)
add_executable(${PROJECT_NAME}_planning2dubs_srv scripts/planning2dubs_srv.cpp)
add_executable(${PROJECT_NAME}_planning2dubs_bench scripts/planning2dubs_bench.cpp)

add_executable(${PROJECT_NAME}_path_exec scripts/utils/path_exec.cpp)
add_executable(${PROJECT_NAME}_path_exec2 scripts/utils/path_exec2.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_laser2ptcloud PROPERTIES OUTPUT_NAME laser2ptcloud PREFIX "")
set_target_properties(${PROJECT_NAME}_octo_proc1 PROPERTIES OUTPUT_NAME octo_proc1 PREFIX "")
set_target_properties(${PROJECT_NAME}_octo_proc2 PROPERTIES OUTPUT_NAME octo_proc2 PREFIX "")
set_target_properties(${PROJECT_NAME}_planning2 PROPERTIES OUTPUT_NAME planning2 PREFIX "")
set_target_properties(${PROJECT_NAME}_planning2_srv PROPERTIES OUTPUT_NAME planning2_srv PREFIX "")
set_target_properties(${PROJECT_NAME}_planning2dubs PROPERTIES OUTPUT_NAME planning2dubs PREFIX "")
set_target_properties(${PROJECT_NAME}_planning2dubs_srv PROPERTIES OUTPUT_NAME planning2dubs_srv PREFIX "")
set_target_properties(${PROJECT_NAME}_planning2dubs_bench PROPERTIES OUTPUT_NAME planning2dubs_bench PREFIX "")

set_target_properties(${PROJECT_NAME}_path_exec PROPERTIES OUTPUT_NAME path_exec PREFIX "")
set_target_properties(${PROJECT_NAME}_path_exec2 PROPERTIES OUTPUT_NAME path_exec2 PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(${PROJECT_NAME}_laser2ptcloud ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_octo_proc1 ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_octo_proc2 ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_planning2 ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_planning2_srv ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_planning2dubs ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_planning2dubs_srv ${LIBRARIES})
target_link_libraries(${PROJECT_NAME}_planning2dubs_bench ${LIBRARIES})

target_link_libraries(${PROJECT_NAME}_path_exec ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_path_exec2 ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/clustering.py
  scripts/clustering_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
