#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

std_msgs::Empty emptyMsg;
geometry_msgs::Twist twistMsgHover;
geometry_msgs::Twist twistMsgTranslate;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motricity");
	ros::NodeHandle node;
	ros::Rate loopRate(50);

	double startTime;

	twistMsgHover.linear.x = 0.0;
	twistMsgHover.linear.y = 0.0;
	twistMsgHover.linear.z = 0.0;
	twistMsgHover.angular.x = 0.0;
	twistMsgHover.angular.y = 0.0;
	twistMsgHover.angular.z = 0.0;

	twistMsgTranslate.linear.x = 0.0;
	twistMsgTranslate.linear.y = 0.1;
	twistMsgTranslate.linear.z = 0.0;
	twistMsgTranslate.angular.x = 0.0;
	twistMsgTranslate.angular.y = 0.0;
	twistMsgTranslate.angular.z = 0.0;

	float takeOffTime = 5.0;
	float flyTime = 7.0;
	float landTime = 3.0;

	ros::Publisher pubEmptyTakeOff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher pubEmptyLand = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher pubTwist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	startTime = static_cast<double>(ros::Time::now().toSec());
	ROS_INFO("Starting motricity at time=%f", startTime);	

	while (ros::ok())
	{
		while (static_cast<double>(ros::Time::now().toSec()) < startTime+takeOffTime)
		{
			pubEmptyTakeOff.publish(emptyMsg);
			pubTwist.publish(twistMsgHover);
			ROS_INFO("Taking Off");
			ros::spinOnce();
			loopRate.sleep();
		}
		while (static_cast<double>(ros::Time::now().toSec()) < startTime+takeOffTime+flyTime)
		{
			pubTwist.publish(twistMsgTranslate);
			ROS_INFO("Translation");
			ros::spinOnce();
			loopRate.sleep();
		}
		while (static_cast<double>(ros::Time::now().toSec()) < startTime+takeOffTime+flyTime+landTime)
		{
			ROS_INFO("Landing");
			pubTwist.publish(twistMsgHover);
			pubEmptyLand.publish(emptyMsg);
			ros::spinOnce();
			loopRate.sleep();
		}
		ROS_INFO("Closing Node");
		exit(0);
	}	
}
