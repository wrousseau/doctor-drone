#ifndef GOING_UP_H
#define GOING_UP_H

#include "basicstate.hpp"

#include <geometry_msgs/Twist.h>

extern ros::NodeHandle *nodeP;

struct GoingUp : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		ROS_INFO("Entering : GoingUp");
		this->startTimer();
		
		ros::Rate loopRate(50);

		std_msgs::Empty emptyMsg;
		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x = 0.0;
		twistMsg.linear.y = 0.0;
		twistMsg.linear.z = 0.05;
		twistMsg.angular.x = 0.0;
		twistMsg.angular.y = 0.0;
		twistMsg.angular.z = 0.0;

		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		while (ros::ok() && static_cast<double>(ros::Time::now().toSec()) < getStartTime()+5.0)
		{
			pubTwist.publish(twistMsg);
			ros::spinOnce();
			loopRate.sleep();
		}
		fsm.process_event(flyEvent());
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting : GoingUp");
	}
};

#endif