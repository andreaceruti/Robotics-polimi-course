cmake_minimum_required(VERSION 2.8.3)
project(project1_calibration)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs
	nav_msgs
	message_generation
	message_filters
	)

add_message_files(
 FILES
 MotorSpeed.msg
 )

 
generate_messages(
   DEPENDENCIES
  	std_msgs
  	nav_msgs
 )


## Declare a catkin package
catkin_package( 
	CATKIN_DEPENDS
 	message_runtime
 	std_msgs
 	nav_msgs
 	message_filters
)




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(calibration src/calibration.cpp)
add_dependencies(calibration project1_calibration_generate_messages_cpp)
target_link_libraries(calibration ${catkin_LIBRARIES})
