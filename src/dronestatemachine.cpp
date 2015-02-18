#include "dronestatemachine.hpp"

#include <iostream>

// Transition actions
void DroneStateMachine::exploringFloorAction(exploringFloorEvent const &)
{
	std::cout << "drone::exploringFloorAction" << std::endl;
}

void DroneStateMachine::goingUpAction(goingUpEvent const&)
{
	std::cout << "drone::goingUpAction" << std::endl;
}

void DroneStateMachine::landAction(landEvent const&)
{
	std::cout << "drone::landAction" << std::endl;
}

void DroneStateMachine::flyAction(flyEvent const&)
{
	std::cout << "drone::flyAction" << std::endl;
}

void DroneStateMachine::stopAction(stopEvent const&)
{
	std::cout << "drone::stopAction" << std::endl;
}