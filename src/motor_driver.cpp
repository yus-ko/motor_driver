#include "ros/ros.h"
#include <std_msgs/String.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <iostream>
#include <motor_driver/motor_driver.h>


#define FILE_MOTOREN "/dev/rtmotoren0"
#define FILE_MOTOR_L "/dev/rtmotor_raw_l0"
#define FILE_MOTOR_R "/dev/rtmotor_raw_r0"
#define FILE_COUNT_L "/dev/rtcounter_l0"
#define FILE_COUNT_R "/dev/rtcounter_r0"
#define BUFF_SIZE 256

changeclass::changeclass(){

	sub_cmd = nh.subscribe("/mouse/cmd_vel", 1, &changeclass::callback,this);
	sub_encoder = nh.subscribe("/mouse/encoder", 1, &changeclass::encoder_callback,this);
	pub_pulse = nh.advertise<motor_driver::vel>("/mouse/cmd_pulse",1);

	ros::NodeHandle n("~");
    n.getParam("control_",control_);
	n.getParam("left_P_",left_P_);
	n.getParam("left_I_",left_I_);
	n.getParam("right_P_",right_P_);
	n.getParam("right_I_",right_I_);
	n.getParam("velocity_P_",velocity_P_);
	n.getParam("velocity_I_",velocity_I_);
	n.getParam("angular_P_",angular_P_);
	n.getParam("angular_I_",angular_I_);
	n.getParam("pulse_upper_limit_",pulse_upper_limit_);
	n.getParam("pulse_lower_limit_",pulse_lower_limit_);

	time_start = ros::Time::now();


}

changeclass::~changeclass(){


}

void changeclass::encoder_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	nav_msgs::Odometry tmp = *msg;
	odom = tmp;
	encoder = tmp.twist.twist;
	//std::cout<<"encoder = " << encoder <<std::endl;
	//if (cmd.linear.x != 0.0 || cmd.angular.z != 0.0) rotate_motor();
	rotate_motor();

	motor_driver::vel wheel_pulse;
	wheel_pulse.r = pulse_r;
	wheel_pulse.l = pulse_l;
	pub_pulse.publish(wheel_pulse);
}

void changeclass::start_callback(const std_msgs::Empty::ConstPtr& msg) {


}

void changeclass::callback(const geometry_msgs::Twist::ConstPtr& vel) {
	cmd = *vel;
	sum_error_vel = sum_error_ang = 0;
	//std::cout<<"cmd = " << cmd <<std::endl;
}

void changeclass::motor_drive(char *freq_l, char *freq_r) {

    FILE *motor_l, *motor_r;
    if ((motor_l = fopen(FILE_MOTOR_L, "w")) != NULL &&
        (motor_r = fopen(FILE_MOTOR_R, "w")) != NULL) {
	//ROS_INFO("OK");
        fputs(freq_l, motor_l);
        fputs(freq_r, motor_r);
    }
    fclose(motor_l);
    fclose(motor_r);
}

void changeclass::manage(){
}

void changeclass::rotate_motor(){
	
	double d = 0.0935;	//車輪間隔[m]
	double r = 0.048;	//車輪半径[m]

	ros::Time now = ros::Time::now();

	double roll, pitch, yaw;
	tf::Quaternion quat;
	quaternionMsgToTF(odom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	if (control_=="pid")
	{
		ROS_INFO("cmd_vel = %f, cmd_ang = %f",cmd.linear.x,cmd.angular.z);
		ROS_INFO("encoder_vel = %f, encoder_ang = %f",odom.twist.twist.linear.x,odom.twist.twist.angular.z);
		double error_vel = cmd.linear.x - odom.twist.twist.linear.x;

		double error_ang;
		if (cmd.angular.z > M_PI)
		{
			error_ang = cmd.angular.z-(M_PI*int(cmd.angular.z/M_PI))-M_PI - yaw;
		}
		else if (cmd.angular.z < -M_PI)
		{
			error_ang = cmd.angular.z-(M_PI*int(cmd.angular.z/M_PI))+M_PI - yaw;
		}
		else
		{
			error_ang = cmd.angular.z - yaw;
		}
		
		//if (abs(error_vel) < 0.02) error_vel = 0;
		if (abs(error_ang) < 0.01) error_ang = sum_error_ang = 0;

		ROS_INFO("error_vel = %f, error_ang = %f",error_vel,error_ang);
		
		double delta_time = 0;
		double diff_error_vel = 0;
		double diff_error_ang = 0;
		if (sum)
		{
			double delta_time = now.toSec() - time_pre.toSec();
			sum_error_vel += error_vel*delta_time;
			sum_error_ang += error_ang*delta_time;
			diff_error_vel = error_vel/delta_time;
			diff_error_ang = error_ang/delta_time;
		}
		else
		{
			sum = true;
		}

		time_pre = now;

		if (cmd.linear.x == 0.0 && cmd.angular.z == 0.0) error_vel = sum_error_vel = diff_error_vel = error_ang = sum_error_ang = diff_error_ang = 0;
		
		double cmd_vel_pid = velocity_P_*error_vel + velocity_I_*sum_error_vel + velocity_D_*diff_error_vel;
		double cmd_ang_pid = angular_P_*error_ang + angular_I_*sum_error_ang + angular_D_*diff_error_ang;
		ROS_INFO("cmd_vel_pid = %f, cmd_ang_pid = %f",cmd_vel_pid,cmd_ang_pid);

		double cmd_vl = cmd_vel_pid - (d/2)*cmd_ang_pid;
		double cmd_vr = cmd_vel_pid + (d/2)*cmd_ang_pid;
		ROS_INFO("cmd_vl = %f, cmd_vr = %f",cmd_vl,cmd_vr);

		double cmd_ang_l = cmd_vl/r;
		double cmd_ang_r = cmd_vr/r;
		ROS_INFO("cmd_ang_l = %f, cmd_ang_r = %f",cmd_ang_l,cmd_ang_r);

		pulse_l = 200*cmd_ang_l/M_PI;
		pulse_r = 200*cmd_ang_r/M_PI;
		//pulse_l = (now.toSec()-time_start.toSec())*50+600;
		//pulse_r = (now.toSec()-time_start.toSec())*50+600;
		// pulse_l = -1000;
		// pulse_r = 1000;
	}
	else if (control_=="ff")
	{
		double cmd_vl = cmd.linear.x - (d/2)*cmd.angular.z;
		double cmd_vr = cmd.linear.x + (d/2)*cmd.angular.z;
		ROS_INFO("cmd_vl = %f, cmd_vr = %f",cmd_vl,cmd_vr);

		double cmd_ang_l = cmd_vl/r;
		double cmd_ang_r = cmd_vr/r;
		ROS_INFO("cmd_ang_l = %f, cmd_ang_r = %f",cmd_ang_l,cmd_ang_r);

		pulse_l = 200*cmd_ang_l/M_PI;
		pulse_r = 200*cmd_ang_r/M_PI;
	}
	else
	{
		double cmd_vl = cmd.linear.x - (d/2)*cmd.angular.z;
		double cmd_vr = cmd.linear.x + (d/2)*cmd.angular.z;

		double encoder_vl = encoder.linear.x - (d/2)*encoder.angular.z;
		double encoder_vr = encoder.linear.x + (d/2)*encoder.angular.z;

		double cmd_ang_l = cmd_vl/r;
		double cmd_ang_r = cmd_vr/r;

		double encoder_ang_l = encoder_vl/r;
		double encoder_ang_r = encoder_vr/r;
		
		double error_ang_l = cmd_ang_l - encoder_ang_l;
		double error_ang_r = cmd_ang_r - encoder_ang_r;

		if (sum)
		{
			double delta_time = now.toSec() - time_pre.toSec();
			sum_error_ang_l += error_ang_l*delta_time;
			sum_error_ang_r += error_ang_r*delta_time;
		}
		else
		{
			sum = true;
		}

		time_pre = now;

		double ang_l = left_P_*error_ang_l + left_I_*sum_error_ang_l;
		double ang_r = right_P_*error_ang_r + right_I_*sum_error_ang_r;

		// ang_l =  cmd_vl;
		// ang_r = ang_l + d*encoder.angular.z;

		pulse_l = 200*ang_l/M_PI;
		pulse_r = 200*ang_r/M_PI;
	}

	//左ステッピングモーターのパルス数リミッター
	if (pulse_l > pulse_upper_limit_)
	{
		pulse_l = pulse_upper_limit_;
	}
	else if (pulse_l < -pulse_upper_limit_)
	{
		pulse_l = -pulse_upper_limit_;
	}
	else
	{
				if(pulse_l < pulse_lower_limit_ && pulse_l > 0) pulse_l = pulse_lower_limit_;
		else if(pulse_l > -pulse_lower_limit_ && pulse_l < 0) pulse_l = -pulse_lower_limit_;
	}

	//右ステッピングモーターのパルス数リミッター
	if (pulse_r > pulse_upper_limit_)
	{
		pulse_r = pulse_upper_limit_;
	}
	else if (pulse_r < -pulse_upper_limit_)
	{
		pulse_r = -pulse_upper_limit_;
	}
	else
	{
				if(pulse_r < pulse_lower_limit_ && pulse_r > 0) pulse_r = pulse_lower_limit_;
		else if(pulse_r > -pulse_lower_limit_ && pulse_r < 0) pulse_r = -pulse_lower_limit_;
	}

	ROS_INFO("pulse_l = %f, pulse_r = %f",pulse_l,pulse_r);

	char str1[BUFF_SIZE],str2[BUFF_SIZE];
	snprintf(str1,BUFF_SIZE,"%d",int(pulse_l));
	puts(str1);
	snprintf(str2,BUFF_SIZE,"%d",int(pulse_r));
	puts(str2);
	motor_drive(str1,str2);

}

int main(int argc, char **argv) {


	ros::init(argc, argv, "motor_driver_md");

	int motoren = open("/dev/rtmotoren0", O_WRONLY);
	printf("Motor On\n");
    write(motoren, "1", 1);

	changeclass ccc;
	ros::spin();

	ccc.motor_drive("0", "0");

	printf("Motor Off\n");
    write(motoren, "0", 1);

    close(motoren);
	
	return 0;
}
