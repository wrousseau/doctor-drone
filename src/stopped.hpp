#ifndef STOPPED_H
#define STOPPED_H

#include "basicstate.hpp"

struct Stopped : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		ROS_INFO("Entering: Stopped");
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting: Stopped");
	}
};

#endif