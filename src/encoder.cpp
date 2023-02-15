#include "ros/ros.h"
#include <std_msgs/String.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <iostream>    
#include <motor_driver/beego_encoder.h>
#include <motor_driver/encoder.h>

#define FILE_MOTOREN "/dev/rtmotoren0"
#define FILE_MOTOR_L "/dev/rtmotor_raw_l0"
#define FILE_MOTOR_R "/dev/rtmotor_raw_r0"
#define FILE_COUNT_L "/dev/rtcounter_l0"
#define FILE_COUNT_R "/dev/rtcounter_r0"
#define BUFF_SIZE 256

encoder::encoder(){

	pub = nh.advertise<nav_msgs::Odometry>("/mouse/encoder",1);
	pub_beego = nh.advertise<motor_driver::beego_encoder>("encoder",1);
	sub_pulse = nh.subscribe("/mouse/cmd_pulse", 1, &encoder::pulse_callback,this);

	odom.pose.pose.orientation.x = 0;
	odom.pose.pose.orientation.y = 0;
	odom.pose.pose.orientation.z = 0;
	odom.pose.pose.orientation.w = 1;
}
encoder::~encoder(){
}

void encoder::pulse_callback(const motor_driver::vel& msg) {
	wheel_palse = msg;
	//ROS_INFO("l:%f, r:%f\n", wheel_palse.l, wheel_palse.r);
	calculate();
}

void encoder::delete_newline(char *str) {
    char *p;

    if ((p = strchr(str, '\n')) != NULL) {
        *p = '\0';
    }
}

void encoder::reset_counters() {
    FILE *count_l, *count_r;

    //ROS_INFO("Reset counter\n");
    if ((count_l = fopen(FILE_COUNT_L, "w")) != NULL &&
        (count_r = fopen(FILE_COUNT_R, "w")) != NULL) {
        fputs("0", count_l);
        fputs("0", count_r);
    }
    fclose(count_l);
    fclose(count_r);
}

void encoder::calculate(){
	FILE *count_l, *count_r;
    char buff_l[BUFF_SIZE];
    char buff_r[BUFF_SIZE];

	ros::Rate rate(5); // ROS Rate at 5Hz

	int L1,R1;
	reset_counters();
	ros::Time start = ros::Time::now();
	usleep(100 * 1000);
	if ((count_l = fopen(FILE_COUNT_L, "r")) != NULL &&
		(count_r = fopen(FILE_COUNT_R, "r")) != NULL) {
		while (fgets(buff_l, BUFF_SIZE, count_l) != NULL) {
		}
		while (fgets(buff_r, BUFF_SIZE, count_r) != NULL) {
		}
		delete_newline(buff_l);
		delete_newline(buff_r);
		//ROS_INFO("count_l:%s, count_r:%s\n", buff_l, buff_r);
		//std::cout<<"buff_l,buff_r = "<<buff_l<< "," << buff_r <<std::endl;
		L1 = atoi(buff_l);
		R1 = atoi(buff_r);
		//ROS_INFO("count_l:%s, count_r:%s\n", L1, R1);
	}
	fclose(count_l);
	fclose(count_r);
	ros::Time end = ros::Time::now();
	double deltatime = end.toSec() - start.toSec();
	//ROS_INFO("L1 = %d, R1 = %d",L1,R1);

	motor_driver::beego_encoder enco;
	std_msgs::Header header;
	header.frame_id = "base_link";
	header.seq = 0;
	header.stamp = ros::Time::now();
	enco.header = header;
	
	enco.acc.x = 0;
	enco.acc.y = 0;
	enco.acc.z = 0;

	odom.header = header;
	double vl = 0;
	double vr = 0;
	double theta = 0;
	if(!isinf(L1_pre) && !isinf(R1_pre))
	{
		
		//reset_counters();
		
		double d = 0.0935;
		double r = 0.048;

		L1 *= wheel_palse.l/abs(wheel_palse.l);
		R1 *= wheel_palse.r/abs(wheel_palse.r);
		// ROS_INFO("l:%f, r:%f", wheel_palse.l, wheel_palse.r);
		// ROS_INFO("L1 = %d, R1 = %d",L1,R1);

		double ang_l = ((L1)*2*M_PI/400)/deltatime;
		double ang_r = ((R1)*2*M_PI/400)/deltatime;
		//ROS_INFO("ang_l = %f, ang_r = %f",ang_l,ang_r);
		vl = ang_l*r;
		vr = ang_r*r;
		double vel = (vl+vr)/2;
		double ang = (-vl+vr)/d;
		//ROS_INFO("vel = %f, ang = %f",vel,ang);

		enco.vol = vel;

		enco.vel.l = L1;//palse
		enco.vel.r = R1;//palse

		double deltatime = header.stamp.toSec() - header_pre.stamp.toSec();

		double roll, pitch, yaw;
		tf::Quaternion quat;
		quaternionMsgToTF(odom.pose.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		
		theta = yaw + ang * deltatime;
		//ROS_INFO("theta=%f",theta/M_PI*180);

		odom.twist.twist.linear.x = vel;
		odom.twist.twist.angular.z = ang;

		odom.pose.pose.position.x += vel * cos(theta);
		odom.pose.pose.position.y += vel * sin(theta);

		quat = tf::createQuaternionFromRPY(roll,pitch,theta);
		quaternionTFToMsg(quat, odom.pose.pose.orientation);
		
	}

	ROS_INFO("robot state");
	std::cout<<"x,y,theta = "<< odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << theta <<std::endl;
	std::cout<<"vel,ang = "<< odom.twist.twist.linear.x << ", " << odom.twist.twist.angular.z <<std::endl;
	std::cout<<"wheel_vel_L,wheel_vel_R = "<< vl << ", " << vr <<std::endl;

	pub.publish(odom);
	pub_beego.publish(enco);

	L1_pre=L1;
	R1_pre=R1;
	header_pre = header;
	//rate.sleep();
	
	

	//error = v - double(L1/(ros::Time::now().toSec() - start.toSec()));
	
}

int main(int argc, char **argv) {


	ros::init(argc, argv, "motor_driver_encoder");
	encoder en;
	usleep(3000 * 1000);
	en.calculate();
	ros::spin();
	en.reset_counters();
	
	return 0;
}
