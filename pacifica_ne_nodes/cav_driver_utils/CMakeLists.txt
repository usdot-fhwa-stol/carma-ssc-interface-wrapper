
# Copyright (C) 2017-2021 LEIDOS.
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
project(cav_driver_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

find_package(catkin REQUIRED COMPONENTS
  bondcpp
  can_msgs
  cav_msgs
  cav_srvs
  roscpp
  std_msgs
)

find_package(Boost REQUIRED system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driver_application cav_socketcan_interface ros_socketcan_interface driver_wrapper
  CATKIN_DEPENDS bondcpp can_msgs cav_msgs cav_srvs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(driver_application
    src/driver_application/driver_application.cpp
)

add_dependencies(driver_application ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(driver_application ${catkin_LIBRARIES})

add_library(cav_socketcan_interface
        src/socketcan_interface/socketcan_interface.cpp
)

add_dependencies(cav_socketcan_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(cav_socketcan_interface ${Boost_LIBRARIES})

add_library(ros_socketcan_interface
        src/ros_socketcan_bridge/ros_socketcan_bridge.cpp
)

add_dependencies(ros_socketcan_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_socketcan_interface ${Boost_LIBRARIES})

add_library(driver_wrapper
    src/driver_wrapper/driver_wrapper.cpp
)

add_dependencies(driver_wrapper ${catkin_EXPORTED_TARGETS})

target_link_libraries(driver_wrapper ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS driver_application driver_wrapper cav_socketcan_interface ros_socketcan_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/cav_driver_utils/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} 
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cav_driver_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
