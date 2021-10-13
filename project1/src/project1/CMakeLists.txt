cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs
	nav_msgs
	message_generation
	message_filters
	tf
	rospy
	dynamic_reconfigure
	)

add_message_files(
 FILES
 MotorSpeed.msg
 CustomOdometry.msg
 )

 add_service_files(
 	FILES
 	SetPose.srv
 	ResetPose.srv
 )
 
generate_messages(
   DEPENDENCIES
  	std_msgs
  	nav_msgs
 )

generate_dynamic_reconfigure_options(
 	cfg/dynamic_rec.cfg
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


add_executable(odometry_executable src/odometry_calculator.cpp)
add_dependencies(odometry_executable project1_generate_messages_cpp)
add_dependencies(odometry_executable ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry_executable ${catkin_LIBRARIES})
