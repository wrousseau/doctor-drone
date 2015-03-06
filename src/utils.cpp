#include <time.h>
#include "utils.hpp"

int Utils::pictureNumber = 0;
std::string Utils::experimentTimestamp = "";
int Utils::currentFloor = 0;
int Utils::windowsPhotographed = 0;

int Utils::getPictureNumber()
{
	return Utils::pictureNumber++;
}

int Utils::getCurrentFloor()
{
	return Utils::currentFloor;
}

int Utils::getWindowsPhotographed()
{
	return Utils::windowsPhotographed;
}

void Utils::incrementCurrentFloor()
{
	Utils::currentFloor++;
	Utils::windowsPhotographed = 0;
}

void Utils::generateExperimentTimestamp()
{
	time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    std::string str(buf);
	Utils::experimentTimestamp = str;
}

std::string Utils::getExperimentTimestamp()
{
	return experimentTimestamp;
}