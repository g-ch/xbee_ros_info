cmake_minimum_required(VERSION 2.8.3)
project(xbee_ros_info)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(

)


include_directories(
        ${catkin_INCLUDE_DIRS}
        src/)

add_library(seriallib_c
        src/serial.h
        src/serial.c)

add_executable(xbee_receive_info src/serial.c src/xbee_receive_info.cpp)
target_link_libraries(xbee_receive_info ${catkin_LIBRARIES} seriallib_c)

add_executable(xbee_send_info src/serial.c src/xbee_send_info.cpp)
target_link_libraries(xbee_send_info ${catkin_LIBRARIES} seriallib_c)

add_executable(info_test src/serial.c src/info_test.cpp)
target_link_libraries(info_test ${catkin_LIBRARIES} seriallib_c)
