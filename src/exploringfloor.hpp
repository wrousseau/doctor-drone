#include <iostream>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include "events.hpp"

namespace
{
    // the drone state machine contains a state which is himself a state machine
    struct ExploringFloor_ : public boost::msm::front::state_machine_def<ExploringFloor_>
    {

        template <class Event, class FSM>
        void on_entry(Event const&, FSM& ) 
        {
            std::cout << "entering: ExploringFloor" << std::endl;
        }
        template <class Event, class FSM>
        void on_exit(Event const&, FSM& ) 
        {
            std::cout << "leaving: ExploringFloor" << std::endl;
        }
        
        // The list of FSM states
        struct Calibrating : public boost::msm::front::state<>
        { 
            template <class Event, class FSM>
            void on_entry(Event const&, FSM& ) 
            {
                std::cout << "starting: Calibrating" << std::endl;
            }
            
            template <class Event, class FSM>
            void on_exit(Event const&, FSM& ) 
            {
                std::cout << "finishing: Calibrating" << std::endl;
            }
        };

        struct Flying : public boost::msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const&, FSM& ) 
            {
                std::cout << "starting: Flying" << std::endl;
            }
            
            template <class Event, class FSM>
            void on_exit(Event const&, FSM& ) 
            {
                std::cout << "finishing: Flying" << std::endl;
            }
        };

        struct Photographing : public boost::msm::front::state<>
        { 
            template <class Event, class FSM>
            void on_entry(Event const&, FSM& ) 
            {
                std::cout << "starting: Photographing" << std::endl;
            }
            
            template <class Event, class FSM>
            void on_exit(Event const&, FSM& ) 
            {
                std::cout << "finishing: Photographing" << std::endl;
            }
        };

        // the initial state. Must be defined
        typedef Calibrating initial_state;
        
        // transition actions
        void flyingAction(flyEvent const&)
        {
            std::cout << "ExploringFloor::flyingAction" << std::endl; 
        }
        void takePicture(windowDetected const&)
        { 
            std::cout << "ExploringFloor::takePicture" << std::endl; 
        }
        void calibrate(calibrationNeeded const&)       
        { 
            std::cout << "ExploringFloor::calibrate" << std::endl;
        }

        typedef ExploringFloor_ ef; // makes transition table cleaner
        // Transition table for Playing
        struct transition_table : boost::mpl::vector4<
            //      Start           Event                Next            Action                Guard
            //    +---------------+--------------------+---------------+---------------------+-------+
        a_row < Calibrating   , flyEvent        , Flying        , &ef::flyingAction           >,
        a_row < Flying        , windowDetected , Photographing , &ef::takePicture    >,
        a_row < Flying        , calibrationNeeded   , Calibrating   , &ef::calibrate      >,
        a_row < Photographing , flyEvent        , Flying        , &ef::flyingAction           >
            //    +---------+-------------+---------+---------------------+----------------------+
        > {};
        // Replaces the default no-transition response.
        template <class FSM,class Event>
        void no_transition(Event const& e, FSM&, int state)
        {
            std::cout << "no transition from state " << state
            << " on event " << typeid(e).name() << std::endl;
        }
    };
}