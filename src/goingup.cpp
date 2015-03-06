#include "goingup.hpp"

void GoingUp::navigationCallback(const ardrone_autonomy::Navdata& msg)
{
	vx = msg.vx * 0.001;
	vy = msg.vy * 0.001;	
	vz = msg.vz * 0.001;	
}

geometry_msgs::Twist GoingUp::computeTwist(double vxTarget, double vyTarget, double vzTarget, double K)
{
	geometry_msgs::Twist twistMsgGen;

	twistMsgGen.linear.x = K*(vxTarget - vx);
	twistMsgGen.linear.y = K*(vyTarget - vy); 
	twistMsgGen.linear.z = K*(vzTarget - vz);
	twistMsgGen.angular.x = 0.0; 
	twistMsgGen.angular.y = 0.0;
	twistMsgGen.angular.z = 0.0;
	ROS_INFO("%f %f %f", twistMsgGen.linear.x, twistMsgGen.linear.y, twistMsgGen.linear.z);
	return twistMsgGen;
}