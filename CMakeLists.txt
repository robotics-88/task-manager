cmake_minimum_required(VERSION 3.5)
project(task_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rcl_interfaces REQUIRED)
find_package(bag_recorder_2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(libmavconn REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(messages_88 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## Specify additional locations of header files
include_directories(
  ${mavlink_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(task_manager SHARED
  src/decco_utilities.cpp
  src/flight_controller_interface.cpp
  src/elevation_manager.cpp
  src/task_manager.cpp
  )

ament_target_dependencies(task_manager
  rclcpp
  rcl_interfaces
  ament_index_cpp
  cv_bridge
  bag_recorder_2
  libmavconn
  livox_ros_driver2
  nav_msgs
  messages_88
  pcl_ros
  std_msgs
  visualization_msgs
)

target_link_libraries(task_manager 
  ${GeographicLib_LIBRARIES}
	${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(task_manager_node
  src/task_manager_node.cpp
)
target_link_libraries(task_manager_node task_manager)

install(TARGETS
  task_manager
  task_manager_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  missions
  dem
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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
  #   src/elevation_manager.cpp
  #   src/flight_controller_interface.cpp
  #   test/task_manager_test.cpp
  # )
  # target_link_libraries(tests_task_manager task_manager)


  # add_rostest_gtest(tests_task_manager
  #   launch/test/tests_task_manager.launch 
  #   src/decco_utilities.cpp
  #   src/task_manager.cpp
  #   src/elevation_manager.cpp
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
  #   src/elevation_manager.cpp 
  #   test/hellodecco_test.cpp
  # )

  # add_rostest_gtest(tests_decco_utilities
  #   launch/test/tests_decco_utilities.launch
  #   src/decco_utilities.cpp
  #   test/decco_utilities_test.cpp
  # )

endif()

ament_export_include_directories(include)

ament_package()
