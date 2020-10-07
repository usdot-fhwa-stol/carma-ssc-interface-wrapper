cmake_minimum_required(VERSION 2.8.3)
project(ssc_pm_cascadia_white)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  geometry_msgs
  roscpp
  pacmod_msgs
  message_filters
)

file(GLOB msg_files msg/*.msg)
set(msgs )
foreach(filePath ${msg_files})
  get_filename_component(name ${filePath} NAME)
  set(msgs ${msgs} ${name})
endforeach(filePath)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ${msgs}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs automotive_platform_msgs automotive_navigation_msgs geometry_msgs pacmod_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(FILES 
  bin/speed_model
  bin/steering_model
  bin/veh_controller
  bin/veh_interface
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
