#ifndef FLYING_H
#define FLYING_H

#include "basicstate.hpp"
#include "events.hpp"

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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
			image_transport::ImageTransport it(*nodeP);
  			image_transport::Subscriber sub = it.subscribe("ardrone/front/image_raw", 1, &Flying::imageCallback, this);
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

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
	    }
	}
};

#endif