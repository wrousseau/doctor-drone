#include <boost/msm/front/state_machine_def.hpp>
#include <typeinfo>

#include "events.hpp"
#include "flying.hpp"
#include "photographing.hpp"

// the drone state machine contains a state which is himself a state machine
struct ExploringFloorStateMachine : public boost::msm::front::state_machine_def<ExploringFloorStateMachine>
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

    // the initial state. Must be defined
    typedef Flying initial_state;

    // Transition actions
    void flyingAction(flyEvent const&);

    void takePicture(windowDetected const&);

    typedef ExploringFloorStateMachine ef; // makes transition table cleaner
    // Transition table for Playing
    struct transition_table : boost::mpl::vector2<
        //      Start           Event                Next            Action                Guard
        //    +-----------+-------------------+---------------+---------------------+-------+
    a_row < Flying        , windowDetected    , Photographing , &ef::takePicture            >,
    a_row < Photographing , flyEvent          , Flying        , &ef::flyingAction           >
        //    +-----------+-------------------+---------------+---------------------+-------+
    > {};

    // Replaces the default no-transition response.
    template <class FSM,class Event>
    void no_transition(Event const& e, FSM&, int state)
    {
        std::cout << "no transition from state " << state
        << " on event " << typeid(e).name() << std::endl;
    }
};