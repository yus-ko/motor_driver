//多重インクルード防止
//#ifndef INCLUDE_CHANGE_CLASS
//#define INCLUDE_CHANGE_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//self msg
#include <motor_driver/beego_encoder.h>
#include <motor_driver/robot_odm.h>
#include <motor_driver/vel.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class changeclass{
    	private:
	double pulse_l = 0, pulse_r = 0;
	motor_driver::beego_encoder enco;
	geometry_msgs::Twist encoder,cmd;
	nav_msgs::Odometry odom;
	ros::NodeHandle nh;
	ros::Subscriber sub_cmd, sub_encoder;
	ros::Publisher pub_pulse;
	ros::Time time_pre, time_start;
	double sum_error_ang_l = 0,sum_error_ang_r = 0, sum_error_vel = 0,sum_error_ang = 0;
	bool sum = false;
	std::string control_;
	double left_P_,left_I_,right_P_,right_I_,velocity_P_,velocity_I_,velocity_D_,angular_P_,angular_I_,angular_D_,pulse_upper_limit_,pulse_lower_limit_;
	public:
	void callback(const geometry_msgs::Twist::ConstPtr& vel);
	void start_callback(const std_msgs::Empty::ConstPtr& msg);
	void encoder_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void motor_drive(char *freq_l, char *freq_r);
	void manage();
	void rotate_motor();
	changeclass();
	~changeclass();
};
