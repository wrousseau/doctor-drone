#ifndef PHOTOGRAPHING_H
#define PHOTOGRAPHING_H

#include "basicstate.hpp"
#include "events.hpp"

struct Photographing : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		ROS_INFO("Entering : Photographing");
		if (isWindowOpened())
		{
			takePicture();
		}
		else
			ROS_INFO("Closed window. Aborting taking picture.");
		fsm.process_event(flyEvent());
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting: Photographing");
	}

	bool isWindowOpened();
	void takePicture();
};

#endif