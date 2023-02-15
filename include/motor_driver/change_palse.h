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
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <iostream>
#include <string.h>
//#include <motor_driver/beego_encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class changeclass{
    	private:
	int L1,R1;
	double pulse_l = 0, pulse_r = 0;
	double v,w,L,error = std::numeric_limits<double>::infinity();
	double dist,outputv,outputw = std::numeric_limits<double>::infinity();
	motor_driver::beego_encoder enco;
	geometry_msgs::Twist be_vel,encoder,cmd;
	nav_msgs::Odometry odom;
	double gyro_x,gyro_y,gyro_z;
	double pgyro_x=0,pgyro_y=0,pgyro_z=0;
	ros::NodeHandle nh;
	ros::NodeHandle nh2;
	ros::NodeHandle publish;
	ros::Subscriber sub, sub_start, sub_encoder;
	ros::Publisher pub, pub_pulse;
	ros::Time time_pre, time_start;
	double sum_error_ang_l = 0,sum_error_ang_r = 0, sum_error_vel = 0,sum_error_ang = 0;
	bool sum = false;
	std::string control_;
	double left_P_,left_I_,right_P_,right_I_,velocity_P_,velocity_I_,angular_P_,angular_I_,pulse_upper_limit_,pulse_lower_limit_;
	public:
	void callback(const geometry_msgs::Twist::ConstPtr& vel);
	void start_callback(const std_msgs::Empty::ConstPtr& msg);
	void encoder_callback(const nav_msgs::Odometry::ConstPtr& msg);
	double change1(double v, double w, double L);
	double change2(double v, double w, double L);
	double hosei_r(double v, double w, double L);
	double repair_v(double palse_r, double palse_l, double L);
	double repair_w(double palse_r, double palse_l, double L);
	void motor_drive(char *freq_l, char *freq_r);
	void delete_newline(char *str);
	void print_counter(const int timeout);
	void reset_counters_and_motors(void);
	void reset_counters(void);
	void manage();
	void rotate_motor();
	double get_error();
	changeclass();
	~changeclass();
};
