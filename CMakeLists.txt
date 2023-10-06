cmake_minimum_required(VERSION 3.0.2)
project(hrov_martech2023)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_STANDARD 11)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
#   set(CMAKE_CXX_EXTENSIONS OFF)
#   # this next line shouldn't be necessary, but doesn't always get added by cmake (e.g., for clang++-5)
#   add_definitions(-std=c++11)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}")
  
include(macros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# set(ompl_DIR /home/srealper/theLibs/ompl/build/Release/)
# set(OMPL_LIBRARIES /home/srealper/theLibs/ompl/build/Release/lib/libompl.so)
find_package(ompl REQUIRED)
message("ompl print")
message("OMPL found? ${OMPL_FOUND}")
# message("1 ${OMPL_FOUND}")
# message("2 ${OMPL_INCLUDE_DIRS}")
# message("3 ${OMPL_LIBRARY_DIRS}")
# message("4 ${OMPL_LIBRARY_DIR}")
# message("5 ${OMPL_LIBRARIES}")
message("${OMPL_VERSION}")

find_package(fcl REQUIRED)
message("fcl prints")
message("fcl found? ${FCL_FOUND}")
# message("1 ${fcl_FOUND}")
# message("2 ${FCL_INCLUDE_DIRS}")
# message("3 ${FCL_LIBRARY_DIRS}")
# message("4 ${FCL_LIBRARIES}")
# message("5 ${FCL_VERSION}")

find_package(gmm_registration)
message("gmm_res prints")
message("gmm_res found? ${gmm_registration_FOUND}")
find_package(PythonLibs 3.8 REQUIRED)

find_package(behaviortree_cpp_v3)
message("BT prints")

find_package(OpenCV REQUIRED)
message("OpenCV version ${OpenCV_VERSION}")

list(APPEND CATKIN_SOURCES
    roscpp
    rospy
    nav_msgs
    tf
    tf2_ros
    eigen_conversions
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
    shape_msgs
    trajectory_msgs
    rosbag
    urdf
    srdfdom
    pluginlib
    geometric_shapes
    tf2_ros
    xmlrpcpp
  
    rviz_visual_tools

    girona_utils
    iauv_kinematic_control
    
    cv_bridge
    image_transport
    image_geometry
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation actionlib genmsg)

add_message_files(
  FILES
  PointArray.msg
  BaseGoal.msg
)

add_service_files(
  FILES
  PlanGoal.srv
)

add_action_files(
  DIRECTORY action 
  FILES PID.action)

catkin_python_setup()

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
 actionlib_msgs
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
  ${PYTHON_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
  )

include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
    ${OMPL_LIBRARIES}
    ${FCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/bt/print.cpp
  src/bt/waitForMsg.cpp
  src/bt/followPath.cpp
  src/bt/planPath.cpp
)

add_script(laser2ptcloud scripts/utils)
add_script(octo_proc1 scripts/legacy)
add_script(octo_proc2 scripts/legacy)
add_script(octo_proc3 scripts)
# add_script(planning2 scripts)
# add_script(planning2_srv scripts)
# add_script(planning2dubs scripts)
# add_script(planning2dubs_srv scripts)
# add_script(planning2dubs_bench scripts)
# add_script(planningR_srv scripts)
add_script(planning3dubs_srv scripts)
add_script(odom2tf scripts/utils)

add_script(clustering scripts)
add_script(clustering_bt scripts)
add_script(aruco_pose scripts)

target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3  ${catkin_LIBRARIES})
add_executable(tree_main scripts/bt/tree_main.cpp)
add_dependencies(tree_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tree_main ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_script(tree_main scripts/bt)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination


catkin_install_python(PROGRAMS
  scripts/clustering.py
  # scripts/clustering_plot.py
  scripts/sm/sm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
