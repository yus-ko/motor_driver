#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <random>

int main(int argc, char **argv) {

    ros::init(argc, argv, "motor_driver_pa");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mouse/cmd_vel",1);

	double publish_frequency_;
	bool publish_random_;
	ros::NodeHandle n("~");
	n.getParam("publish_frequency_",publish_frequency_);
	n.getParam("publish_random_",publish_random_);

    double velocities[5];
	double rotations[5];
	double actions[26][4];
	
	double v0 = 0;
	double v_pref = 0.6;
	
	for(int i=0;i<5;i++){
		double r = double(i)/4 * M_PI / 3 - M_PI / 6;
		rotations[i] = -r;
	}
	
	for(int i=0;i<5;i++){
		double v = v_pref * ( double(i) + 1 ) / 5;
		velocities[i] = v;
	}
	
	// 速度、角度
	int cnt = 0;
	for(int i = 0; i < 5; i++){
		for(int j=0;j<5;j++){
			actions[cnt][0] = velocities[i];
			actions[cnt][1] = rotations[j];
			//actions[cnt][2] = 0.5-double(cnt)*0.01;
			cnt = cnt + 1;
		}
	}
	actions[25][0] = actions[25][1] = v0;

    std::random_device rand;
    ros::Rate rate(publish_frequency_);
    int idx = 0;
    double yaw = 0;
    while (ros::ok())
    {
		if (publish_random_)
		{
			idx = rand() % 26;
		}
        
        geometry_msgs::Twist cmd;
        yaw += actions[idx][1];
        cmd.linear.x = actions[idx][0];
        cmd.angular.z = yaw;
        if (idx == 25) cmd.angular.z = 0;
        pub.publish(cmd);

		if (!publish_random_)
		{
			idx++;
			if (idx > 25) idx = 0;
		}

        rate.sleep();
    }

    return 0;
}