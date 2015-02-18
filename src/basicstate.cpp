#include "basicstate.hpp"
#include <iostream>

double BasicState::getStartTime() const
{
	return startTime;
}

void BasicState::setStartTime(const double& startTime)
{
	this->startTime = startTime;
}

void BasicState::startTimer()
{
	std::cout << "Starting timer" << std::endl;
	startTime = static_cast<double>(ros::Time::now().toSec());
}