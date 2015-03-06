#ifndef GOING_UP_H
#define GOING_UP_H

#include "basicstate.hpp"
#include "events.hpp"

#include <ardrone_autonomy/Navdata.h>

#include <geometry_msgs/Twist.h>

extern ros::NodeHandle *nodeP;

struct GoingUp : public BasicState
{
private:
	double vx;
	double vy;
	double vz;
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		ROS_INFO("Entering : GoingUp");
		this->startTimer();
		
		ros::Rate loopRate(50);

		double speed = 0.75;

		geometry_msgs::Twist twistTarget;
		twistTarget.linear.x = 0.0;
		twistTarget.linear.y = 0.0;
		twistTarget.linear.z = speed;
		twistTarget.angular.x = 0.0;
		twistTarget.angular.y = 0.0;
		twistTarget.angular.z = 0.0;
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;

		double K = 0.75;

		ros::Subscriber navigationSub = nodeP->subscribe("/ardrone/navdata", 1, &GoingUp::navigationCallback, this);	
		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		while (ros::ok() && static_cast<double>(ros::Time::now().toSec()) < getStartTime()+5.0)
		{
			geometry_msgs::Twist computedTwist = computeTwist(twistTarget.linear.x, twistTarget.linear.y, twistTarget.linear.z, K);
			pubTwist.publish(computedTwist);			
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

	geometry_msgs::Twist computeTwist(double vxTarget, double vyTarget, double vzTarget, double K);
	void navigationCallback(const ardrone_autonomy::Navdata& msg);
};

#endif