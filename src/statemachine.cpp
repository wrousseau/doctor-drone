#include <ros/ros.h>
#include <boost/msm/back/state_machine.hpp>

#include "dronestatemachine.hpp"
#include "utils.hpp"
#include <signal.h>

ros::NodeHandle *nodeP;

// Pick a back-end
typedef boost::msm::back::state_machine<DroneStateMachine> Drone;


int main(int argc, char** argv)
{
	Utils::generateExperimentTimestamp();
	ros::init(argc, argv, "statemachine");
	ros::NodeHandle node;
	nodeP = &node;
	Drone drone;
	drone.start();
	exit(0);
}
