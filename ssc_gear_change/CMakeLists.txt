cmake_minimum_required(VERSION 2.8.3)
project(ssc_gear_change)

add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

find_package(
  catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gear_change
  src/gear_change/gear_change_node.cpp
  src/gear_change/gear_change.cpp
)

target_link_libraries(gear_change ${catkin_LIBRARIES})

add_dependencies(gear_change ${catkin_EXPORTED_TARGETS})


install(TARGETS gear_change
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Uncomment if launch files are added
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )
