
# Copyright (C) 2020-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 2.8.3)
project(ssc_pm_cascadia_dot_10004)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()


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
