#include "ros/ros.h"
#include <std_msgs/String.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <iostream>    
//#include <motor_driver/beego_encoder.h>
#include <motor_driver/change_palse.h>


#define FILE_MOTOREN "/dev/rtmotoren0"
#define FILE_MOTOR_L "/dev/rtmotor_raw_l0"
#define FILE_MOTOR_R "/dev/rtmotor_raw_r0"
#define FILE_COUNT_L "/dev/rtcounter_l0"
#define FILE_COUNT_R "/dev/rtcounter_r0"
#define BUFF_SIZE 256

changeclass::changeclass(){

	sub = nh.subscribe("/beego/cmd_vel", 1, &changeclass::callback,this);
	pub = nh2.advertise<motor_driver::beego_encoder>("encoder",1);

}

changeclass::~changeclass(){


}

void changeclass::callback(const geometry_msgs::Twist::ConstPtr& vel) {
	
	v = vel -> linear.x; 
	w = vel -> angular.z;
	//ROS_INFO("v :%f",v);
	manage();
}


//zisaku kansu

double changeclass::change1(double v, double w, double L){

	double vr = v + (L * w)/2;//速度right
	// double vl = v - (L * w)/2;//速度left
	
	double k_r = vr/(0.048 * M_PI);//回転数right
	// double k_l = vl/(0.048 * M_PI);//回転数left
	
	double palse_r = k_r * 400;//入力信号数right
	// double palse_l = k_l * 400;//入力信号数left
	//printf("palse_r :%f\n",palse_r);

	return palse_r;
}

double changeclass::change2(double v, double w, double L){

	// double vr = v + (L * w)/2;//速度right
	double vl = v - (L * w)/2;//速度left
	
	// double k_r = vr/(0.048 * M_PI);//回転数right
	double k_l = vl/(0.048 * M_PI);//回転数left
	
	// double palse_r = k_r * 400;//入力信号数right
	double palse_l = k_l * 400;//入力信号数left

	return palse_l;

}

double changeclass::hosei_r(double v, double w, double L){

	//double vr = v + (L * w)/2;//速度right
	double vl = v - (L * w)/2;//速度left

	double e = outputw - w;

	double vr = vl + L * e;//hosei速度right

	double k_r = vr/(0.048 * M_PI);//回転数right
	
	double palse_r = k_r * 400;//入力信号数right

	return palse_r;
}

double changeclass::repair_v(double palse_r, double palse_l, double L){

	double kr_re = palse_r / 400;
	double vr_re = kr_re * (0.048 * M_PI);

	double kl_re = palse_l / 400;
	double vl_re = kl_re * (0.048 * M_PI);

	double velocity = (vr_re + vl_re) / 2;//入力信号数right
	
	return velocity;
}

double changeclass::repair_w(double palse_r, double palse_l, double L){

	double kr_re = palse_r / 400;
	double vr_re = kr_re * (0.048 * M_PI);

	double kl_re = palse_l / 400;
	double vl_re = kl_re * (0.048 * M_PI);

	double omega = (vr_re - vl_re) / L;//入力信号数right
	
	return omega;
}


//dekiai kansu

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

void changeclass::delete_newline(char *str) {
    char *p;

    if ((p = strchr(str, '\n')) != NULL) {
        *p = '\0';
    }
}


void changeclass::print_counter(const int timeout) {
    FILE *count_l, *count_r;
    char buff_l[BUFF_SIZE];
    char buff_r[BUFF_SIZE];

    time_t start = time(NULL);
    int elapsed_time = 0;

    while (elapsed_time < timeout) {
        if ((count_l = fopen(FILE_COUNT_L, "r")) != NULL &&
            (count_r = fopen(FILE_COUNT_R, "r")) != NULL) {
            while (fgets(buff_l, BUFF_SIZE, count_l) != NULL) {
            }
            while (fgets(buff_r, BUFF_SIZE, count_r) != NULL) {
            }
            delete_newline(buff_l);
            delete_newline(buff_r);
            //ROS_INFO("count_l:%s, count_r:%s\n", buff_l, buff_r);
			std::cout<<"buff_l,buff_r = "<<buff_l<< "," << buff_r <<std::endl;
			L1 = atoi(buff_l);
			R1 = atoi(buff_r);
			dist = repair_v(R1,L1,L);
			if(dist > v){
			// fclose(count_l);
			// fclose(count_r);
			break;
	    }   

        }
        fclose(count_l);
        fclose(count_r);

        elapsed_time = (int)(time(NULL) - start);
    }
}

void changeclass::reset_counters_and_motors(void) {
    FILE *count_l, *count_r;

    char a[1],b[1];
    std::sprintf(a, "%d",0);
    std::sprintf(b, "%d",0);

    motor_drive(a, b);

    ROS_INFO("Reset counter\n");
    if ((count_l = fopen(FILE_COUNT_L, "w")) != NULL &&
        (count_r = fopen(FILE_COUNT_R, "w")) != NULL) {
        fputs("0", count_l);
        fputs("0", count_r);
    }
    fclose(count_l);
    fclose(count_r);
}

void changeclass::manage(){

	ROS_INFO("V :%f , W :%f",v,w);
	//FILE *motoren = fopen("/dev/rtmotoren0", "w");
	int motor_l = open("/dev/rtmotoren0",O_WRONLY);
	//入力値を持ってくる
	//v = 0.01;
	//w = M_PI/6;
	L = 0.0935;
	//

	int c1 = int(change1(v,w,L));//right
	int c2 = int(change2(v,w,L));//left
	
	// int c1 = 800;//right
	// int c2 = 800;//left

	ROS_INFO("c1 :%d , c2 :%d",c1,c2);
	
	double w_cmd = 0;
	if (!isinf(outputw))
	{
		//double w_input = w;
		//double Kp = 1;
		//w_cmd = w_input + Kp*(w_input-outputw);
		c1 = int(hosei_r(v,w,L));//right
		ROS_INFO("c1 :%d ",c1);
	}
	
	//ROS_INFO("V :%f , w_cmd :%f",v,w_cmd);

	//int error1 = (c1-R1);
	//int error2 = (c2-L1);
	//ROS_INFO("error1 :%d , error2 :%d",error1,error2);

	char str1[BUFF_SIZE],str2[BUFF_SIZE];
	ROS_INFO("c1 :%d , c2 :%d",c1,c2);
	snprintf(str1,BUFF_SIZE,"%d",c1);
	puts(str1);
	snprintf(str2,BUFF_SIZE,"%d",c2);
	puts(str2);

	//ROS_INFO("Motor On");
	//fwrite("1",1, 1, motoren);
	write(motor_l, "1", 1);

	//printf("a :%s\n",str1);

	//ROS_INFO("Rotate left motor");
	//usleep(500 * 1000);
	motor_drive(str2, str1);
	//motor_drive("400", "400");
	print_counter(10);

	reset_counters_and_motors();
	//ROS_INFO("L1 :%d , R1 :%d",L1,R1);
	outputv = repair_v(R1,L1,L);
	outputw = repair_w(R1,L1,L);
	ROS_INFO("return V :%f , return W :%f",outputv,outputw);
	ROS_INFO("return R1 :%d , return L1 :%d",R1,L1);

	//enco.heade;
	std_msgs::Header header;
	header.frame_id = "base_link";
	header.seq = 0;
	header.stamp = ros::Time::now();
	enco.header = header;

	enco.vol = outputv;


	//enco.gyro.x = gyro_x + pgyro_x;
	//enco.gyro.y = gyro_y + pgyro_y;
	//enco.gyro.z = gyro_z + pgyro_z;
	enco.acc.x = 0;
	enco.acc.y = 0;
	enco.acc.z = 0;
	enco.vel.l = L1;//palse
	enco.vel.r = R1;//palse	

	//printf("Motor Off\n");
	write(motor_l, "0", 1);

	close(motor_l);
	
	pgyro_x = enco.gyro.x;
	pgyro_y = enco.gyro.y;
	pgyro_z = enco.gyro.z;

	pub.publish(enco);
}


int main(int argc, char **argv) {


	ros::init(argc, argv, "motor_driver_node");

	changeclass ccc;


	ros::spin();
	return 0;
}
