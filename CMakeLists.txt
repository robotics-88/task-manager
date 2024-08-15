cmake_minimum_required(VERSION 3.5)
project(task_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp_action)
# find_package(actionlib_msgs)
find_package(bag_recorder)
find_package(geometry_msgs)
find_package(livox_ros_driver2)
find_package(mavros)
find_package(mavros_msgs)
find_package(messages_88)
find_package(pcl_ros)
find_package(sensor_msgs)
find_package(std_msgs)
find_package(tf2)
find_package(tf2_geometry_msgs)
find_package(tf2_ros)

find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## Specify additional locations of header files
include_directories(
  ${GeographicLib_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  include
)

add_executable(task_manager
  src/decco_utilities.cpp
  src/flight_controller_interface.cpp
  src/hello_decco_manager.cpp
  src/task_manager.cpp
  src/task_manager_node.cpp
)

ament_target_dependencies(task_manager
  messages_88
  pcl_ros
  rclcpp
  std_msgs
  
)

install(TARGETS
  task_manager
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(task_manager 
  ${GeographicLib_LIBRARIES}
	${PCL_LIBRARIES}
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  
  find_package(ament_cmake_gtest REQUIRED)

  # ament_add_gtest(${PROJECT_NAME}_test_services test/test_services.cpp)
  # target_include_directories(${PROJECT_NAME}_test_services PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  

  # ament_add_gtest(tests_task_manager
  #   launch/test/tests_task_manager.launch 
  #   src/decco_utilities.cpp
  #   src/task_manager.cpp
  #   src/hello_decco_manager.cpp
  #   src/flight_controller_interface.cpp
  #   test/task_manager_test.cpp
  # )
  # target_link_libraries(tests_task_manager task_manager)


  # add_rostest_gtest(tests_task_manager
  #   launch/test/tests_task_manager.launch 
  #   src/decco_utilities.cpp
  #   src/task_manager.cpp
  #   src/hello_decco_manager.cpp
  #   src/flight_controller_interface.cpp
  #   test/task_manager_test.cpp
  # )

  # add_rostest_gtest(tests_flight_controller_interface
  #   launch/test/tests_flight_controller_interface.launch
  #   src/flight_controller_interface.cpp 
  #   test/flight_controller_interface_test.cpp
  # )

  # add_rostest_gtest(tests_hellodecco
  #   launch/test/tests_hellodecco.launch
  #   src/decco_utilities.cpp
  #   src/hello_decco_manager.cpp 
  #   test/hellodecco_test.cpp
  # )

  # add_rostest_gtest(tests_decco_utilities
  #   launch/test/tests_decco_utilities.launch
  #   src/decco_utilities.cpp
  #   test/decco_utilities_test.cpp
  # )

endif()
  
ament_package()
