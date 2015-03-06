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
#include "windowinspector/WindowInspector.h"

extern ros::NodeHandle *nodeP;

class Flying : public BasicState
{
private:
	OpticalFlowEqualizer *opticalFlowEqualizer;
	WindowInspector *inspector;
	double vx;
	double vy;
	double vz;
	cv::Mat currentImage;
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{		
		ROS_INFO("Entering : Flying");

		// Initializing image processing related objects
		cv::Size size(320, 240);
		opticalFlowEqualizer = new OpticalFlowEqualizer(size);
		inspector = new WindowInspector();

		// Initializing the speed vector and the K coefficient
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;
		double K = 0.75;

		// The drone is to move left for even floors (0, 2)
		int direction = (Utils::getCurrentFloor() % 2 == 0) ? 1 : -1; 
		double speed = direction * 0.75;

		geometry_msgs::Twist twistTarget;
		twistTarget.linear.x = 0.0;
		twistTarget.linear.y = speed;
		twistTarget.linear.z = 0.0;
		twistTarget.angular.x = 0.0;
		twistTarget.angular.y = 0.0;
		twistTarget.angular.z = 0.0;

		ros::Rate loopRate(50);

		// Publisher for the twist vector
		ros::Publisher pubTwist = nodeP->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		// Subscriber for the drone navigation data
		ros::Subscriber navigationSub = nodeP->subscribe("/ardrone/navdata", 1, &Flying::navigationCallback, this);	
		// Subscriber for the front camera of the drone
		image_transport::ImageTransport it(*nodeP);
  		image_transport::Subscriber imageSub = it.subscribe("/ardrone/image_raw", 1, &Flying::imageCallback, this);
		
		// Thereshold to know when to move up (after a certain amount of detected windows)
		int windowsThreshold = 5;
		// Thereshold to know when to stop the experiment (after a certain amount of processed floors)
		int floorsThreshold = 3;
		// Simple counter to make sure that the same window is not photographed several times
		int i = 0;

		while (ros::ok())
		{
			// If all windows were processed for that floor
			if (Utils::getWindowsPhotographed() >= windowsThreshold)
			{
				// If we were on the last floor
				if (Utils::getCurrentFloor() >= floorsThreshold)
				{
					// The drone lands
					ROS_INFO("Mission Accomplished");
					fsm.process_event(landEvent());
					break;
				}
				else
				{
					// The drone goes up
					// TODO : Make sure the drone does not go to high
					ROS_INFO("Going Up");
					Utils::incrementCurrentFloor();
					//fsm.process_event(goingUpEvent());
					break;
				}
			}
			else if (isThereAWindow(i)) // If a window is detected on the current image
			{
				ROS_INFO("Window detected");
				fsm.process_event(windowDetected());
				break;
			}
			else // Else the drone moves
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
		delete inspector;
	}

	// Checks if there is a window (except if one was too recently detected)
	bool isThereAWindow(int i);

	// Computing the twist vector (P controller)
	geometry_msgs::Twist computeTwist(double vxTarget, double vyTarget, double vzTarget, double K);

	// ROS subscriber callbacks
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void navigationCallback(const ardrone_autonomy::Navdata& msg);
};

#endif