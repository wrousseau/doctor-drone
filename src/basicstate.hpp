#ifndef BASIC_STATE_H
#define BASIC_STATE_H

#include <boost/msm/front/state_machine_def.hpp>
#include <ros/ros.h>

/*
 * A basic state machine state with a timer
 */
struct BasicState : public boost::msm::front::state<>
{
private:
	double startTime;
public:
	double getStartTime() const;
	void setStartTime(const double &startTime);
	void startTimer();
};

#endif