cmake_minimum_required(VERSION 2.8.3)
project(bw_auto_dock)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  galileo_serial_server
)

add_service_files(
  FILES
  Charge.srv
  StopCharge.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# fix cannot link liborocos-kdl.so
list(REMOVE_DUPLICATES catkin_LIBRARIES)
list(REMOVE_ITEM catkin_LIBRARIES "/opt/ros/kinetic/lib/liborocos-kdl.so" "/opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0")

set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)

# fix cannot link liborocos-kdl.so
list(REMOVE_ITEM catkin_LIBRARIES "/opt/ros/kinetic/lib/liborocos-kdl.so" "/opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0")

set(SRCS src/main.cpp src/AsyncSerial.cpp src/DockController.cpp src/StatusPublisher.cpp src/getDockPosition.cpp)
add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")
# fix build deps
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
