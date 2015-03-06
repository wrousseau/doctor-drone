#ifndef DRONE_STATE_MACHINE_H
#define DRONE_STATE_MACHINE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <ros/ros.h>

#include "events.hpp"
#include "takingoff.hpp"
#include "goingup.hpp"
#include "landing.hpp"
#include "stopped.hpp"
#include "flying.hpp"
#include "photographing.hpp"


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

	// Initial state of the Drone
	typedef TakingOff initial_state;

	// Transition actions
	void goingUpAction(goingUpEvent const&);

	void landAction(landEvent const&);

	void stopAction(stopEvent const&);

	void flyingAction(flyEvent const&);

    void takePicture(windowDetected const&);

	// Makes transition table cleaner
	typedef DroneStateMachine d; 

	// Transition table for drone
	struct transition_table : boost::mpl::vector<
		//    Start            Event                 Next             Action                     Guard
		//  +----------------+---------------------+----------------+--------------------------+--------+
	a_row   < TakingOff      , flyEvent            , Flying         , &d::flyingAction                  >,
	a_row   < Flying         , windowDetected      , Photographing  , &d::takePicture                   >,
	a_row   < Flying         , goingUpEvent        , GoingUp        , &d::goingUpAction                 >,
	a_row   < Flying         , landEvent           , Landing        , &d::landAction                    >,
	a_row   < Photographing  , flyEvent            , Flying         , &d::flyingAction                  >,
	a_row   < GoingUp        , flyEvent            , Flying         , &d::flyingAction                  >,
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