#include "exploringfloor.hpp"

namespace
{
	// Front-end (defining the FSM structure)
	struct drone_ : public boost::msm::front::state_machine_def<drone_>
	{
		template <class Event, class FSM>
		void on_entry(Event const&, FSM&)
		{
			std::cout << "Entering: Drone" << std::endl;
		}
		template <class Event, class FSM>
		void on_exit(Event const&, FSM&)
		{
			std::cout << "Leaving: Drone" << std::endl;
		}

		// The list of FSM states
		struct TakingOff : public boost::msm::front::state<>
		{
			template <class Event, class FSM>
			void on_entry(Event const&, FSM&)
			{
				std::cout << "Entering Taking Off" << std::endl;
			}
			template <class Event, class FSM>
			void on_exit(Event const&, FSM&)
			{
				std::cout << "Exiting Taking Off" << std::endl;
			}
		};

		struct Flying : public boost::msm::front::state<>
		{
			template <class Event, class FSM>
			void on_entry(Event const&, FSM&)
			{
				std::cout << "Entering Flying" << std::endl;
			}
			template <class Event, class FSM>
			void on_exit(Event const&, FSM&)
			{
				std::cout << "Exiting Flying" << std::endl;
			}
		};

        // back-end
        typedef boost::msm::back::state_machine<ExploringFloor_> ExploringFloor;

        struct GoingUp : public boost::msm::front::state<>
		{
			template <class Event, class FSM>
			void on_entry(Event const&, FSM&)
			{
				std::cout << "Entering GoingUp" << std::endl;
			}
			template <class Event, class FSM>
			void on_exit(Event const&, FSM&)
			{
				std::cout << "Exiting GoingUp" << std::endl;
			}
		};

		struct Landing : public boost::msm::front::state<>
		{
			template <class Event, class FSM>
			void on_entry(Event const&, FSM&)
			{
				std::cout << "Entering Landing" << std::endl;
			}
			template <class Event, class FSM>
			void on_exit(Event const&, FSM&)
			{
				std::cout << "Exiting Landing" << std::endl;
			}
		};

				// sm_ptr still supported but deprecated as functors are a much better way to do the same thing
        struct Stopped : public boost::msm::front::state<boost::msm::front::default_base_state,boost::msm::front::sm_ptr> 
        {	 
            template <class Event,class FSM>
            void on_entry(Event const& ,FSM&)
            {
            	std::cout << "entering: Stopped" << std::endl;
            }
            template <class Event,class FSM>
            void on_exit(Event const&,FSM& ) 
            {
            	std::cout << "leaving: Stopped" << std::endl;
            }
            void set_sm_ptr(drone_* dr)
            {
                m_drone=dr;
            }
            drone_* m_drone;
        };

		// Initial state of the Drone
		typedef TakingOff initial_state;

		// Transition actions
		void exploringFloorAction(exploringFloorEvent const &)
		{
			std::cout << "drone::exploringFloorAction" << std::endl;
		}

		void goingUpAction(goingUpEvent const&)
		{
			std::cout << "drone::goingUpAction" << std::endl;
		}

		void landAction(landEvent const&)
		{
			std::cout << "drone::landAction\n";
		}

		void flyAction(flyEvent const&)
		{
			std::cout << "drone::flyAction\n";
		}

		void stopAction(stopEvent const&)
		{
			std::cout << "drone::stopAction\n";
		}
		
		// Makes transition table cleaner
		typedef drone_ d; 

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

	// Pick a back-end
	typedef boost::msm::back::state_machine<drone_> drone;

	//
	// Testing utilities.
	//
	static char const* const state_names[] = { "TakingOff", "ExploringFloor", "GoingUp", "Landing", "Stopped"};
	void dstate(drone const& d)
	{
		std::cout << " -> " << state_names[d.current_state()[0]] << std::endl;
	}

	void test()
	{
		drone d;
		d.start();
		// Go to exploring floor phase
		d.process_event(exploringFloorEvent()); dstate(d);
		d.process_event(flyEvent()); dstate(d);
		d.process_event(calibrationNeeded()); dstate(d);
		d.process_event(flyEvent()); dstate(d);
		d.process_event(windowDetected()); dstate(d);
		d.process_event(flyEvent()); dstate(d);

		d.process_event(goingUpEvent()); dstate(d);
		d.process_event(exploringFloorEvent()); dstate(d);
		d.process_event(flyEvent()); dstate(d);

		d.process_event(goingUpEvent()); dstate(d);
		d.process_event(exploringFloorEvent()); dstate(d);
		d.process_event(flyEvent()); dstate(d);

		// Go to Landing
		d.process_event(landEvent()); dstate(d);
		d.process_event(stopEvent()); dstate(d);
		std::cout << "stop fsm" << std::endl;
	}
}

int main(int argc, char** argv)
{
	test();
	return 0;
}
