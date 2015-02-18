#ifndef GOING_UP_H
#define GOING_UP_H

#include "basicstate.hpp"

struct GoingUp : public BasicState
{
public:
	template <class Event, class FSM>
	void on_entry(Event const& e, FSM& fsm)
	{
		ROS_INFO("Entering : GoingUp");
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		ROS_INFO("Exiting : GoingUp");
	}
};

#endif