#include "dronestatemachine.hpp"

#include <iostream>

void DroneStateMachine::goingUpAction(goingUpEvent const&)
{
	std::cout << "drone::goingUpAction" << std::endl;
}

void DroneStateMachine::landAction(landEvent const&)
{
	std::cout << "drone::landAction" << std::endl;
}

void DroneStateMachine::flyingAction(flyEvent const&)
{
	std::cout << "drone::flyingAction" << std::endl;
}

void DroneStateMachine::stopAction(stopEvent const&)
{
	std::cout << "drone::stopAction" << std::endl;
}

void DroneStateMachine::takePicture(windowDetected const&)
{ 
    std::cout << "drone::takePicture" << std::endl; 
}