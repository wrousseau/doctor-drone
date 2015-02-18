#ifndef DRONE_STATE_MACHINE_H
#define DRONE_STATE_MACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <ros/ros.h>

#include "events.hpp"
#include "takingoff.hpp"
#include "exploringfloorstatemachine.hpp"
#include "goingup.hpp"
#include "landing.hpp"
#include "stopped.hpp"

struct DroneStateMachine : public boost::msm::front::state_machine_def<DroneStateMachine>
{

	template <class Event, class FSM>
	void on_entry(Event const&, FSM&)
	{
		std::cout << "Entering: DroneStateMachine" << std::endl;
	}

	template <class Event, class FSM>
	void on_exit(Event const&, FSM&)
	{
		std::cout << "Leaving: DroneStateMachine" << std::endl;
	}

    // Back-end
	typedef boost::msm::back::state_machine<ExploringFloorStateMachine> ExploringFloor;

	// Initial state of the Drone
	typedef TakingOff initial_state;

	// Transition actions
	void exploringFloorAction(exploringFloorEvent const &);

	void goingUpAction(goingUpEvent const&);

	void landAction(landEvent const&);

	void flyAction(flyEvent const&);

	void stopAction(stopEvent const&);

	// Makes transition table cleaner
	typedef DroneStateMachine d; 

	// Transition table for drone
	struct transition_table : boost::mpl::vector<
		//    Start            Event                 Next             Action                     Guard
		//  +----------------+---------------------+----------------+--------------------------+--------+
	a_row   < TakingOff      , exploringFloorEvent , ExploringFloor , &d::exploringFloorAction          >,
	a_row   < ExploringFloor , goingUpEvent        , GoingUp        , &d::goingUpAction                 >,
	a_row   < ExploringFloor , landEvent           , Landing        , &d::landAction                    >,
	a_row   < GoingUp        , exploringFloorEvent , ExploringFloor , &d::exploringFloorAction          >,
	a_row   < Landing        , stopEvent           , Stopped        , &d::stopAction                    >,
	_row    < Stopped        , stopEvent           , Stopped                                            >
	> {};

	// Replaces the default no-transition response.
	template <class FSM, class Event>
	void no_transition(Event const& e, FSM&, int state)
	{
		std::cout << "No transition from state " << state << " on event " << typeid(e).name() << std::endl;
	}
};

#endif