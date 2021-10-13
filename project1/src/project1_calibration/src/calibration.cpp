//standard includes
#include "ros/ros.h"
#include "project1/MotorSpeed.h"

//include geometry_msgs
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

//include for message_filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <math.h>

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.1575
#define REAL_BASELINE 0.583

#define GEAR_RATIO_BAG1 38.212963
#define GEAR_RATIO_BAG2 38.248463
#define GEAR_RATIO_BAG3 38.217575

#define APPARENT_BASELINE_BAG1 1.035883
#define APPARENT_BASELINE_BAG2 1.028561
#define APPARENT_BASELINE_BAG3 1.029486




typedef message_filters::sync_policies
      ::ApproximateTime<project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed, nav_msgs::Odometry> MySyncPolicy;

float total_gear_ratio = 0;
int count = 0;

float total_apparent_baseline = 0;

void callback(const project1::MotorSpeed::ConstPtr& msg_fl, const project1::MotorSpeed::ConstPtr& msg_fr,
			  const project1::MotorSpeed::ConstPtr& msg_rl, const project1::MotorSpeed::ConstPtr& msg_rr,
			  const nav_msgs::Odometry::ConstPtr& scout_odom)
{

	/*double linear_velocity = scout_odom->twist.twist.linear.x;

	double gear_ratio = (msg_fr->rpm + msg_rr->rpm - msg_fl->rpm - msg_rl->rpm)*M_PI*WHEEL_RADIUS/(60*linear_velocity*2);

	if (gear_ratio > 35 && gear_ratio < 40 && (linear_velocity != 0)){
		total_gear_ratio += gear_ratio;
		count ++;

		ROS_INFO("gear_ratio calculated is %f, avg gear ratio is %f at %d iteration", gear_ratio, total_gear_ratio/count, count);
	}*/

	double angular_velocity = scout_odom->twist.twist.angular.z;

	double appararent_baseline = (msg_fr->rpm + msg_rr->rpm + msg_fl->rpm + msg_rl->rpm)*(M_PI*WHEEL_RADIUS)/(angular_velocity*60*GEAR_RATIO_BAG1);

	if(angular_velocity != 0 && appararent_baseline < 2 && appararent_baseline > -2){

		if (appararent_baseline < 0)
			appararent_baseline = -1*appararent_baseline;

		total_apparent_baseline += appararent_baseline;
		count ++;

		ROS_INFO("baseline %f, avg baseline is %f at %d iteration", appararent_baseline, total_apparent_baseline/count, count);
	}


}

int main(int argc, char**argv){
	ros::init(argc, argv, "calibration_node");


	ros::NodeHandle sub_node;
	message_filters::Subscriber<project1::MotorSpeed> sub_fl(sub_node, "motor_speed_fl", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_fr(sub_node, "motor_speed_fr", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_rl(sub_node, "motor_speed_rl", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_rr(sub_node, "motor_speed_rr", 1);
	message_filters::Subscriber<nav_msgs::Odometry> sub_scout(sub_node, "scout_odom", 1);

	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr, sub_scout);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));


	ros::spin();
	return 0;
}