#ifndef FLYING_H
#define FLYING_H

#include "basicstate.hpp"
#include "events.hpp"

#include <geometry_msgs/Twist.h>

extern ros::NodeHandle *nodeP;

class Flying : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{		
		ROS_INFO("Entering : Flying");

		ros::Rate loopRate(50);
		double speed = 0.01;

		geometry_msgs::Twist twistMsg;
		twistMsg.linear.x = 0.0;
		twistMsg.linear.y = speed;
		twistMsg.linear.z = 0.0;
		twistMsg.angular.x = 0.0;
		twistMsg.angular.y = 0.0;
		twistMsg.angular.z = 0.0;

		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		while (ros::ok())
		{
			if (isThereAWindow())
				fsm.process_event(windowDetected());
			else if (isFloorFinished())
				fsm.process_event(goingUpEvent());
			else 
			{
				twistMsg.angular.z = getStabilisatingAngle();
				pubTwist.publish(twistMsg);
			}
			ros::spinOnce();
			loopRate.sleep();
		}
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting : Flying");
	}

	double getStabilisatingAngle();
	bool isThereAWindow();
	bool isFloorFinished();
};

#endif