cmake_minimum_required(VERSION 2.8.3)
project(task_manager)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  bag_recorder
  geometry_msgs
  livox_ros_driver
  mavros_msgs
  messages_88
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    std_msgs
)

###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++114support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  include
)

add_executable(task_manager
  src/drone_state_manager.cpp
  src/hello_decco_manager.cpp
  src/task_manager.cpp
  src/task_manager_node.cpp
)
#add_dependencies(monarch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(monarch)

target_link_libraries(task_manager 
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
	${PCL_LIBRARY_DIRS}
)

#############
## Install ##
#############

# install nodes
install(TARGETS task_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)

  find_package(rostest REQUIRED)

  add_rostest_gtest(tests_drone_state
    launch/test/tests_drone_state.launch 
    src/drone_state_manager.cpp 
    test/drone_state_manager_test.cpp
  )
  target_link_libraries(tests_drone_state 
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(tests_takeoff
    launch/test/tests_takeoff.launch 
    src/drone_state_manager.cpp 
    test/drone_takeoff_test.cpp
  )
  target_link_libraries(tests_takeoff 
    ${catkin_LIBRARIES}
  )
endif()
