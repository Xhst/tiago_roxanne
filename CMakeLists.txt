cmake_minimum_required(VERSION 3.0.2)
project(tiago_roxanne)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    dynamic_reconfigure
    aruco_ros
    actionlib
    actionlib_msgs
    std_srvs
    std_msgs
    geometry_msgs
    pal_interaction_msgs
    moveit_msgs
    roxanne_rosjava_msgs
)  

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    TimerToken.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    ModelPose.srv
    permission.srv
    Position.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  
    geometry_msgs
    roxanne_rosjava_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    roxanne_rosjava_msgs
)


catkin_install_python(PROGRAMS 
    scripts/tiago_robot.py 
    scripts/tiago_input.py 
    scripts/timer_result.py
    scripts/tiago_roxanne_start.py 
    scripts/services/position_service.py 
    scripts/services/model_pose_service.py 
    scripts/services/permission_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
