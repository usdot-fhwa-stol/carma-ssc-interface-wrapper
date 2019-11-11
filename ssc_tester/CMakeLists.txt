cmake_minimum_required(VERSION 2.8.3)
project(ssc_tester)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

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
