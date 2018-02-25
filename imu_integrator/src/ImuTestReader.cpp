#include "ros/ros.h"
#include "std_msgs/String.h"
#include <msg_definitions/imu_data.h>



void imu_dataCallback(const msg_definitions::imu_data::ConstPtr& msg)
{	
	char roll_string[33], pitch_string[10], yaw_string[10];
	snprintf(roll_string, 10, "%f", msg->roll);
	snprintf(pitch_string, 10, "%f", msg->pitch);
	snprintf(yaw_string, 10, "%f", msg->yaw);
	strcat(roll_string, " ");
	strcat(roll_string, pitch_string);
	strcat(roll_string, " ");
	strcat(roll_string, yaw_string);	
	ROS_INFO("IMU Reader read: [%s]", roll_string);	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImuTestReader");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("imu_data", 1000, imu_dataCallback);
	ROS_INFO("Reader has started");
	ros::spin();
	return 0;
	
}