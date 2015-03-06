#ifndef FLYING_H
#define FLYING_H

#include "basicstate.hpp"
#include "events.hpp"
#include "utils.hpp"

#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ardrone_autonomy/Navdata.h>

#include <boost/msm/back/state_machine.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opticalflowequalizer/OpticalFlowEqualizer.h"

extern ros::NodeHandle *nodeP;

class Flying : public BasicState
{
private:
	OpticalFlowEqualizer *opticalFlowEqualizer;
	double vx;
	double vy;
	double vz;
	cv::Mat currentImage;
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{		
		ROS_INFO("Entering : Flying");

		cv::Size size(320, 240);
		opticalFlowEqualizer = new OpticalFlowEqualizer(size);

		ros::Rate loopRate(50);
		int direction = (Utils::getCurrentFloor() % 2 == 0) ? 1 : -1; 
		double speed = direction * 0.75;

		geometry_msgs::Twist twistTarget;
		twistTarget.linear.x = 0.0;
		twistTarget.linear.y = speed;
		twistTarget.linear.z = 0.0;
		twistTarget.angular.x = 0.0;
		twistTarget.angular.y = 0.0;
		twistTarget.angular.z = 0.0;
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;

		double K = 0.75;

		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		ros::Subscriber navigationSub = nodeP->subscribe("/ardrone/navdata", 1, &Flying::navigationCallback, this);	

		image_transport::ImageTransport it(*nodeP);
  		image_transport::Subscriber imageSub = it.subscribe("/ardrone/image_raw", 1, &Flying::imageCallback, this);
		
		int i = 0;
		int threshold = 5;
		while (ros::ok())
		{
			if (Utils::getWindowsPhotographed() >= threshold)
			{
				ROS_INFO("Going Up");
				Utils::incrementCurrentFloor();
				fsm.process_event(goingUpEvent());
			}
			else if (isThereAWindow(i))
			{
				ROS_INFO("Window detected");
				fsm.process_event(windowDetected());
				break;
			}
			else 
			{
				geometry_msgs::Twist computedTwist = computeTwist(twistTarget.linear.x, twistTarget.linear.y, twistTarget.linear.z, K);
				pubTwist.publish(computedTwist);
			}
			i++;
			ros::spinOnce();
			loopRate.sleep();
		}
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting : Flying");
		delete opticalFlowEqualizer;
	}

	double getStabilisatingAngle();
	bool isThereAWindow(int i);
	geometry_msgs::Twist computeTwist(double vxTarget, double vyTarget, double vzTarget, double K);

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	void navigationCallback(const ardrone_autonomy::Navdata& msg);
};

#endif