//standard includes
#include "ros/ros.h"
#include "project1/MotorSpeed.h"
#include "project1/CustomOdometry.h"
#include <math.h>

//publishers includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

//message_filters includes
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//tf broadcaster includes
#include "tf/transform_broadcaster.h"

//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <project1/dynamic_recConfig.h>

//services includes
#include "project1/SetPose.h"
#include "project1/ResetPose.h"

#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 0.1575
#define REAL_BASELINE 0.583

//estimated gear ratio and apparent baseline with /scout_odom topic for every bag
#define GEAR_RATIO_BAG1 38.212963		
#define APPARENT_BASELINE_BAG1 1.035883 

#define GEAR_RATIO_BAG2 38.212963		
#define APPARENT_BASELINE_BAG2 1.035883 

#define GEAR_RATIO_BAG3 38.212963		
#define APPARENT_BASELINE_BAG3 1.035883 

//typedef for the message filters Time policy
typedef message_filters::sync_policies
      ::ApproximateTime<project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed> MySyncPolicy;

//struct used to put info about motor messages coming from the bag topics 
typedef struct input_data
{
	double motor_rpm_fl;
	double motor_rpm_fr;
	double motor_rpm_rl;
	double motor_rpm_rr;

	double time_fl;
	double time_fr;
	double time_rl;
	double time_rr;
}Data;

//struct used to store linear and angular velocity calculated at each time
typedef struct velocities{
	double linear_velocity;
	double angular_velocity;
}Velocity;

//struct used to store the pose of the scout robot
typedef struct pose{
	double x;
	double y;
	double theta;
}Pose;

//class used to store the principal information of the robot (Pose, velocity) and to store the publishers required 
class scout_odometry_calculator{

	private:

		ros::NodeHandle pub_node;
		ros::Publisher velocities_pub;		//publisher for velocities messages --> geometry_msgs/TwistStamped
		ros::Publisher odometry_pub;		//publisher for odometry messages --> nav_msgs/Odometry
		ros::Publisher customOdom_pub;		//publisher for the custom odometry messages

		tf::TransformBroadcaster odom_broadcaster;		//publisher for tf

		ros::ServiceServer set_pose_service;
		ros::ServiceServer reset_pose_service;

		Velocity velocities_scout; 
		Pose current_pose;

		//parameters to keep track of pose and time of the scout base, they are used in odometry calculation
		double prev_time;
		double prev_x;
		double prev_y;
		double prev_theta;	

		//parameter for dynamic reconfigure. 0 is euler method, 1 is runge-kutta
		int chosen_odometry_type;

	public:

		scout_odometry_calculator(){
			velocities_pub = pub_node.advertise<geometry_msgs::TwistStamped>("/calculated_velocities", 1);
			odometry_pub = pub_node.advertise<nav_msgs::Odometry>("/calculated_odom", 10);
			customOdom_pub = pub_node.advertise<project1::CustomOdometry>("/simple_odom", 10);

			set_pose_service = pub_node.advertiseService("set_pose", &scout_odometry_calculator::set_pose_service_function, this);
			reset_pose_service = pub_node.advertiseService("reset_pose", &scout_odometry_calculator::reset_pose_service_function, this);

			pub_node.getParam("/initial_x", current_pose.x);
			pub_node.getParam("/initial_y", current_pose.y);
			pub_node.getParam("/initial_theta", current_pose.theta);

			prev_time = 0.0;
			prev_x = current_pose.x;
			prev_y = current_pose.y;
			prev_theta = current_pose.theta;

			chosen_odometry_type = 0;

		}

		bool reset_pose_service_function(project1::ResetPose::Request &req, project1::ResetPose::Response &res){
			prev_x = 0.0;
			prev_y = 0.0;
			prev_theta = 0.0;

			res.response = "pose correctly resetted to (0,0,0)";

			return true;
		}

		bool set_pose_service_function(project1::SetPose::Request &req, project1::SetPose::Response &res ){
			prev_x = req.x;
			prev_y = req.y;
			prev_theta = req.theta;

			ROS_INFO("new pose: x %f, y %f, theta %f", prev_x, prev_y, prev_theta);

			res.response = "new pose set";
			return true;
		}


		//method used to switch between euler and runge kutta integration		
		void set_chosen_odometry(int odometryMethod){
			if (odometryMethod == 0 || odometryMethod == 1){
				chosen_odometry_type = odometryMethod;
			}

		}

		//testing if dynamic reconfigure is working
		int get_odometry_method(){
			return chosen_odometry_type;
		}

		Pose get_current_pose(){
			return current_pose;
		}

		void compute_velocities(Data *input_topics){
			double wheel_linear_velocity_fl = input_topics->motor_rpm_fl * ((2*M_PI*WHEEL_RADIUS) / (60*GEAR_RATIO_BAG1));	//motor_rpm/GEAR_RATIO: wheel rpm
			double wheel_linear_velocity_fr = input_topics->motor_rpm_fr * ((2*M_PI*WHEEL_RADIUS) / (60*GEAR_RATIO_BAG1));	//motor_rps * (2*M_PI / 60): wheel rad/s
			double wheel_linear_velocity_rl = input_topics->motor_rpm_rl * ((2*M_PI*WHEEL_RADIUS) / (60*GEAR_RATIO_BAG1));	//wheel rad/s * WHEEL_RADIUS: wheel m/s
			double wheel_linear_velocity_rr = input_topics->motor_rpm_rr * ((2*M_PI*WHEEL_RADIUS) / (60*GEAR_RATIO_BAG1));

			double velocity_left = -1*(wheel_linear_velocity_fl + wheel_linear_velocity_rl) / 2;
			double velocity_right = (wheel_linear_velocity_fr + wheel_linear_velocity_rr) / 2;

			velocities_scout.linear_velocity = (velocity_right + velocity_left) / 2;
			velocities_scout.angular_velocity = (velocity_right - velocity_left) / APPARENT_BASELINE_BAG1;
		}

		void publish_velocities(geometry_msgs::TwistStamped *calculated_velocities){

			//set header
			ros::Time current_time = ros::Time::now();

			calculated_velocities->header.stamp = current_time;
			calculated_velocities->header.frame_id = "pose"; //TODO

			//set linear velocity
			calculated_velocities->twist.linear.x = velocities_scout.linear_velocity;
			calculated_velocities->twist.linear.y = 0;
			calculated_velocities->twist.linear.z = 0;

			//set angular velocity
			calculated_velocities->twist.angular.x = 0;
			calculated_velocities->twist.angular.z = 0;
			calculated_velocities->twist.angular.z = velocities_scout.angular_velocity;

			velocities_pub.publish(*calculated_velocities);

		}

		void compute_odometry(Data *input_topics){
			if (chosen_odometry_type == 0){
				compute_odometry_euler(input_topics);
			}
			if(chosen_odometry_type == 1){
				compute_odometry_rungekutta(input_topics);
			}

		}

		void compute_odometry_euler(Data *input_topics){

			//time calculation for the sampling time: average of the input motor times
			double current_time = (input_topics->time_fl + input_topics->time_fr + input_topics->time_rl + input_topics->time_rr)/4.0;
			
			double t_sampling = current_time - prev_time;
			prev_time = current_time;

			//euler formulas
			current_pose.x = prev_x + (velocities_scout.linear_velocity * t_sampling * cos(prev_theta));
			prev_x = current_pose.x;

			current_pose.y = prev_y + (velocities_scout.linear_velocity * t_sampling * sin(prev_theta));
			prev_y = current_pose.y;

			current_pose.theta = prev_theta + (velocities_scout.angular_velocity * t_sampling);
			prev_theta = current_pose.theta;
		}

		void compute_odometry_rungekutta(Data *input_topics){

			//time calculation for the sampling time: average of the input motor times
			double current_time = (input_topics->time_fl + input_topics->time_fr + input_topics->time_rl + input_topics->time_rr)/4.0;
			
			double t_sampling = current_time - prev_time;
			prev_time = current_time;

			//runge-kutta formulas
			current_pose.x = prev_x + (velocities_scout.linear_velocity * t_sampling * cos(prev_theta + (velocities_scout.angular_velocity*t_sampling/2.0)));
			prev_x = current_pose.x;

			current_pose.y = prev_y + (velocities_scout.linear_velocity * t_sampling * sin(prev_theta + (velocities_scout.angular_velocity*t_sampling/2.0)));
			prev_y = current_pose.y;

			current_pose.theta = prev_theta + (velocities_scout.angular_velocity * t_sampling);
			prev_theta = current_pose.theta;
		}


		
		void publish_odometry(nav_msgs::Odometry *calculated_odometry, project1::CustomOdometry *custom_odometry){

			ros::Time current_time = ros::Time::now();
			geometry_msgs::TransformStamped odom_trans;
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pose.theta);

			//odometry header
			calculated_odometry->header.stamp = current_time;
			calculated_odometry->header.frame_id = "odom";

			//set position
			calculated_odometry->pose.pose.position.x = current_pose.x;
			calculated_odometry->pose.pose.position.y = current_pose.y;
			calculated_odometry->pose.pose.position.z = 0;
			calculated_odometry->pose.pose.orientation = odom_quat;

			//set velocity
			calculated_odometry->child_frame_id = "base_link";
			calculated_odometry->twist.twist.linear.x = velocities_scout.linear_velocity;
			calculated_odometry->twist.twist.linear.y = 0;
			calculated_odometry->twist.twist.linear.z = 0;

			calculated_odometry->twist.twist.angular.x = 0;
			calculated_odometry->twist.twist.angular.y = 0;
			calculated_odometry->twist.twist.angular.z = velocities_scout.angular_velocity;
			
			//transform
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = current_pose.x;
			odom_trans.transform.translation.y = current_pose.y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom_broadcaster.sendTransform(odom_trans);
			odometry_pub.publish(*calculated_odometry);

			//-----------------------------------------------------------------------------
			//now I fill the attributes of the custom message and I publish it

			if (chosen_odometry_type == 0)
				custom_odometry->method.data = "euler";
			else if (chosen_odometry_type == 1)
				custom_odometry->method.data = "rk";

			//set header
			custom_odometry->odom.header.stamp = current_time;
			custom_odometry->odom.header.frame_id = "odom";

			//set position
			custom_odometry->odom.pose.pose.position.x = current_pose.x;
			custom_odometry->odom.pose.pose.position.y = current_pose.y;
			custom_odometry->odom.pose.pose.position.z = 0;
			custom_odometry->odom.pose.pose.orientation = odom_quat;

			//set velocity
			custom_odometry->odom.twist.twist.linear.x = velocities_scout.linear_velocity;
			custom_odometry->odom.twist.twist.linear.y = 0;
			custom_odometry->odom.twist.twist.linear.z = 0;

			custom_odometry->odom.twist.twist.angular.x = 0;
			custom_odometry->odom.twist.twist.angular.y = 0;
			custom_odometry->odom.twist.twist.angular.z = velocities_scout.angular_velocity;

			customOdom_pub.publish(*custom_odometry);
		}

};

//message filters callback. it takes also as parameter the pointer to our scout class so that we can call the methods inside it
void callback(const project1::MotorSpeed::ConstPtr& msg_fl, const project1::MotorSpeed::ConstPtr& msg_fr,
			  const project1::MotorSpeed::ConstPtr& msg_rl, const project1::MotorSpeed::ConstPtr& msg_rr, 
			  scout_odometry_calculator *scout, Data *input_topics){

	input_topics->motor_rpm_fl = msg_fl->rpm;
	input_topics->time_fl = msg_fl->header.stamp.toSec();
	input_topics->motor_rpm_fr = msg_fr->rpm;
	input_topics->time_fr = msg_fr->header.stamp.toSec();
	input_topics->motor_rpm_rl = msg_rl->rpm;
	input_topics->time_rl = msg_rl->header.stamp.toSec();
	input_topics->motor_rpm_rr = msg_rr->rpm;
	input_topics->time_rr = msg_rr->header.stamp.toSec();

	//compute and publish velocities	
	geometry_msgs::TwistStamped calculated_velocities;
	scout->compute_velocities(input_topics);
	scout->publish_velocities(&calculated_velocities);

	//compute and publish odometry
	nav_msgs::Odometry calculated_odometry;
	project1::CustomOdometry simple_odometry;
	scout->compute_odometry(input_topics);
	scout->publish_odometry(&calculated_odometry, &simple_odometry);

	ROS_INFO("scout pose: x %f, y %f, theta %f, odometry method %d", scout->get_current_pose().x, scout->get_current_pose().y, scout->get_current_pose().theta, scout->get_odometry_method());

}

//dynamic reconfigure callback
void param_callback(project1::dynamic_recConfig &config, uint32_t level, scout_odometry_calculator *scout ){

	scout->set_chosen_odometry(config.odometry_integration_type);

	ROS_INFO("reconfigure request %d", config.odometry_integration_type);
}


int main(int argc, char**argv){

	ros::init(argc, argv, "odometry_calculator_node");
	Data input;

	scout_odometry_calculator *scout = NULL;
	scout = new scout_odometry_calculator();

	//dynamic reconfigure for odometry type
	dynamic_reconfigure::Server<project1::dynamic_recConfig> server;
	dynamic_reconfigure::Server<project1::dynamic_recConfig>::CallbackType f;
	f = boost::bind(&param_callback, _1, _2, scout);
	server.setCallback(f);

	//message filters
	ros::NodeHandle sub_node;
	message_filters::Subscriber<project1::MotorSpeed> sub_fl(sub_node, "motor_speed_fl", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_fr(sub_node, "motor_speed_fr", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_rl(sub_node, "motor_speed_rl", 1);
	message_filters::Subscriber<project1::MotorSpeed> sub_rr(sub_node, "motor_speed_rr", 1);

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, scout, &input));

	ros::spin();

	return 0;
}


