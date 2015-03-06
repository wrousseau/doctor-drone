#ifndef PHOTOGRAPHING_H
#define PHOTOGRAPHING_H

#include "basicstate.hpp"
#include "events.hpp"

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

extern ros::NodeHandle *nodeP;

struct Photographing : public BasicState
{
private:
	bool hasTakenPicture;
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		startTimer();
		hasTakenPicture = false;
		
		ROS_INFO("Entering : Photographing");

		ros::Rate loopRate(50);

		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		// Stopping for 10 seconds
		while (ros::ok() && static_cast<double>(ros::Time::now().toSec()) < getStartTime()+10.0)
		{
			pubTwist.publish(twist);
			ros::spinOnce();
			loopRate.sleep();
		}

		// Taking the picture by subscribing to the drone front camera
		image_transport::ImageTransport it(*nodeP);
  		image_transport::Subscriber imageSub = it.subscribe("/ardrone/image_raw", 1, &Photographing::imageCallback, this);
		while (ros::ok() && !hasTakenPicture)
		{
			pubTwist.publish(twist);
			ros::spinOnce();
			loopRate.sleep();
		}

		fsm.process_event(flyEvent());
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting: Photographing");
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif