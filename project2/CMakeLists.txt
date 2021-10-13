cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(
  catkin REQUIRED
  COMPONENTS
    nav_msgs
    std_msgs
    sensor_msgs
    roscpp
    tf
)



generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    sensor_msgs
)


