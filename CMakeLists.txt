cmake_minimum_required(VERSION 2.8.3)
project(contact_stability)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    roslib
    std_msgs
    message_generation
    geometry_msgs
)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
    FILES
    Contact.msg
)

add_service_files(
    FILES
    StaticArea.srv
    PendularArea.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS 
    rospy
    message_runtime
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)
