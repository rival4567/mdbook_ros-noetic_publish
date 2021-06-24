cmake_minimum_required(VERSION 3.0.2)
project(pkg_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  gazebo_msgs
  nav_msgs
  std_srvs
  dynamic_reconfigure
  gazebo_dev
  gazebo_ros
  geometry_msgs
  roscpp
  rviz
  std_msgs
  tf
  tf2
  tf2_ros
)

find_package(gazebo REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pkg_gazebo
 CATKIN_DEPENDS dynamic_reconfigure gazebo_dev gazebo_ros geometry_msgs roscpp rviz std_msgs tf tf2
 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pkg_gazebo.cpp
# )

#############
## Install ##
#############

#############
## Testing ##
#############