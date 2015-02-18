#ifndef LANDING_H
#define LANDING_H

#include "basicstate.hpp"

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

extern ros::NodeHandle *nodeP;

struct Landing : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		this->startTimer();
		
		ROS_INFO("Entering : Landing");

		ros::Rate loopRate(50);

		std_msgs::Empty emptyMsg;
		geometry_msgs::Twist twistMsgHover;
		twistMsgHover.linear.x = 0.0;
		twistMsgHover.linear.y = 0.0;
		twistMsgHover.linear.z = 0.0;
		twistMsgHover.angular.x = 0.0;
		twistMsgHover.angular.y = 0.0;
		twistMsgHover.angular.z = 0.0;

		ros::Publisher pubEmptyLand = nodeP->advertise<std_msgs::Empty>("/ardrone/land", 1);
		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		while (ros::ok() && static_cast<double>(ros::Time::now().toSec()) < getStartTime()+3.0)
		{
			pubEmptyLand.publish(emptyMsg);
			pubTwist.publish(twistMsgHover);
			ros::spinOnce();
			loopRate.sleep();
		}
		fsm.process_event(exploringFloorEvent());
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting : Landing");
	}
};

#endif