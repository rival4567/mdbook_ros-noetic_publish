cmake_minimum_required(VERSION 3.0.2)
project(pkg_ros_basics)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
    myMessage.msg
)

add_service_files(
    FILES
    AddTwoInts.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg_ros_basics
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)