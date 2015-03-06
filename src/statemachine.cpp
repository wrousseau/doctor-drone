#include <ros/ros.h>
#include <boost/msm/back/state_machine.hpp>
#include <signal.h>

#include "dronestatemachine.hpp"
#include "utils.hpp"

ros::NodeHandle *nodeP;

// Pick a back-end
typedef boost::msm::back::state_machine<DroneStateMachine> Drone;

int main(int argc, char** argv)
{
	// Generating the timestamp for the experiment
	Utils::generateExperimentTimestamp();

	ros::init(argc, argv, "statemachine");
	// A global pointer to the ROS NodeHandle is created
	ros::NodeHandle node;
	nodeP = &node;

	// Creating and starting the state machine 
	Drone drone;
	drone.start();

	exit(0);
}
