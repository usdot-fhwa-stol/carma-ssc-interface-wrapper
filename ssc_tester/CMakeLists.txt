
# Copyright (C) 2019-2021 LEIDOS.
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
project(ssc_tester)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

set (DEPS 
  rospy
  std_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  ssc_ds_fusion
  ssc_interface_wrapper
  ssc_ne_pacifica
  ssc_pm_lexus
  as
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPS}
)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS ${DEPS}
)

catkin_install_python(PROGRAMS src/ssc_tester/ssc_tester_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
