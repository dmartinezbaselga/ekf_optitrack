cmake_minimum_required(VERSION 3.0.2)
project(ekf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    roslib
    std_msgs
    tf2
    tf2_ros 
    message_generation
)


add_message_files(FILES PoseVel.msg PoseVelArray.msg)
generate_messages(DEPENDENCIES geometry_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS roscpp roslib geometry_msgs std_msgs tf2 tf2_ros message_runtime)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(ekf src/ekf.cpp)
add_dependencies(ekf ekf_generate_messages_cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

add_executable(exp_manager src/exp_manager.cpp)
target_link_libraries(exp_manager ${catkin_LIBRARIES})