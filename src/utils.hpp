#ifndef UTILS_H
#define UTILS_H

#include <string>

/*
 * Provides a bunch of static methods which provide
 * data which several states should access or modify
 */
class Utils
{
private:
	static int pictureNumber;
	static std::string experimentTimestamp;
	static int currentFloor;
	static int windowsPhotographed;
public:
    static int getPictureNumber();
    static int getCurrentFloor();
    static int getWindowsPhotographed();
    static void incrementCurrentFloor();
    static void generateExperimentTimestamp();
    static std::string getExperimentTimestamp();
};

#endif