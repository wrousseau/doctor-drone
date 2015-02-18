#include "exploringfloorstatemachine.hpp"

#include <iostream>

void ExploringFloorStateMachine::flyingAction(flyEvent const&)
{
    std::cout << "ExploringFloor::flyingAction" << std::endl; 
}

void ExploringFloorStateMachine::takePicture(windowDetected const&)
{ 
    std::cout << "ExploringFloor::takePicture" << std::endl; 
}