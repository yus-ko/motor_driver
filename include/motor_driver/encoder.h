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

class encoder{
	private:
		std_msgs::Header header_pre;
		motor_driver::beego_encoder enco;
		geometry_msgs::Twist be_vel;
		motor_driver::vel wheel_palse;
		ros::NodeHandle nh, nh2;
		ros::Subscriber sub_pulse;
		ros::Publisher pub, pub_beego;
		int L1_pre = std::numeric_limits<double>::infinity();
		int R1_pre = std::numeric_limits<double>::infinity();
		nav_msgs::Odometry odom;
	public:
		void pulse_callback(const motor_driver::vel& msg);
		void delete_newline(char *str);
		void reset_counters();
		void calculate();
		encoder();
		~encoder();
};
