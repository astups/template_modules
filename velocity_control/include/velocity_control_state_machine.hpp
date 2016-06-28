/*****************************************************************************/
/*                  ASTUPS – velocity_control_state_machine                  */
/*****************************************************************************/
/*
* Author : Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
* Creation Date : 07/02/2015
* License : BSD-3-Clause
*/

#include <state_machine.hpp>
#include <speed_command.hpp>
#include <angle_command.hpp>
#include <config.hpp>
#include <ros/ros.h>
#include <list>

// libvelocity_control objects
namespace car_control
{
	SpeedCommand propulsion = SpeedCommand(max_speed,speed_type,speed_pin,speed_direction_pin);
	AngleCommand wheel = AngleCommand(max_angle,angle_type,angle_pin,angle_direction_pin);
}

//Define the possible events
enum event: unsigned int {start, stop, paused, resume, emergency, reinit};

STATE(Init);
//EMPTY_ONENTRY(Init);
//EMPTY_ONEXIT(Init);
PRINT_ONENTRY(Init);
PRINT_ONEXIT(Init);

STATE(Pause);
//EMPTY_ONENTRY(Pause);
//EMPTY_ONEXIT(Pause);
PRINT_ONENTRY(Pause);
PRINT_ONEXIT(Pause);


STATE(Emergency);
//EMPTY_ONENTRY(Emergency);
//EMPTY_ONEXIT(Emergency);
PRINT_ONEXIT(Emergency);

void Emergency::onEntry(event e)
{
	ROS_INFO("Entering Emergency state. Emergency stop engaged.");
	// TODO: Change namespace

	car_control::propulsion.setSpeed(0);

	ROS_INFO("Emergency stop proceed. Waiting for reinitialisation.");
}

// Redefinition of Run class
// Add mutex for message queue
class Run: public State{
public:
	std::thread* _action_on_state;

	bool _quit_action;
	std::mutex _quit_action_mutex;

	std::list<velocity_control::command> _commandQueue;
	std::mutex _commandQueue_mutex;


	void action();
	void onEntry(event e);
	void onExit(event e);
	bool hasAction()
	{
		return true;
	}

	void startAction()
	{
		//_quit_action_mutex.lock();
		_quit_action=false;
		_quit_action_mutex.unlock();
		_action_on_state=new std::thread(&Run::action, this);
	}

	void pushAction(velocity_control::command cmd)
	{
		_commandQueue_mutex.lock();
		_commandQueue.push_back(cmd);
		_commandQueue_mutex.unlock();
	}

	void stopAction()
	{
		_quit_action_mutex.lock();
		_quit_action=true;
		_quit_action_mutex.unlock();
		if(_action_on_state && std::this_thread::get_id()!=_action_on_state->get_id())
			_action_on_state->join();

		delete _action_on_state;
	}
} Run;

PRINT_ONENTRY(Run);
PRINT_ONEXIT(Run);

void Run::action()
{
	// simulate expensive operation
	std::cout<<"  Start run loop"<<std::endl;
	
	bool quit;
	std::size_t queueSize;

	velocity_control::command cmd;

	// Get quit boolean
	std::cout << "  Waiting a mutex..." << std::endl;
	std::cout.flush();

	_quit_action_mutex.lock();
	quit=_quit_action;
	_quit_action_mutex.unlock();
	
	std::cout << "  Just before the loop" << std::endl;

	while(!quit)
	{
		_commandQueue_mutex.lock();
		queueSize = _commandQueue.size();

		std::cout << "  I Just got my mutex !" << std::endl;

		if ( queueSize != 0 )
		{
			// Pop order
			ROS_INFO("a %lu",_commandQueue.size());
			cmd = _commandQueue.front();
			ROS_INFO("b %lu",_commandQueue.size());
			_commandQueue.pop_front();
			_commandQueue_mutex.unlock();
			
			// Clear list if asked
			if ( cmd.override )
			{
				_commandQueue.clear();
			}

			// Do some PWMs
			if ( !car_control::propulsion.setSpeed(cmd.linear) )
			{
				ROS_INFO("[WARNING] Invalid speed command: wanted %f limit (+/-) %f !",cmd.linear,max_speed);
			}

			if ( !car_control::wheel.setAngle(cmd.angular) )
			{
				ROS_INFO("[WARNING] Invalid angular command: wanted %f limit (+/-) %f !",cmd.angular,max_angle);
			}

			// Wait for next order !
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(cmd.duration)*1000));

		}
		else
		{
			car_control::propulsion.setSpeed(0);
			_commandQueue_mutex.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(25));
			std::cout << "  Queue is empty mate..." << std::endl;
		}
		
		_quit_action_mutex.lock();
		quit=_quit_action;
		_quit_action_mutex.unlock();
	}
	std::cout<<"  Stop run loop"<<std::endl;

}


//Define the initial state
INIT_STATE(Init);

//Define the transition table
const std::map<State*, std::map<event, State*>> StateMachine::_transition_table({
	{&Init, {{start, &Run},
		{emergency, &Emergency}}},
	{&Run, {{paused, &Pause},
		{stop, &Init},
		{emergency, &Emergency}}},
	{&Pause, {{resume, &Run},
		{stop, &Init},
		{emergency, &Emergency}}},
	{&Emergency, {{reinit, &Init}}}
});

//Define the test function
void test(){
	//Run
	state_machine.process(start);
	sleep(2);
	state_machine.process(emergency);

	//Nothing
	state_machine.process(stop);
	state_machine.process(paused);
	state_machine.process(resume);

	//Emergency and back
	state_machine.process(emergency);
	state_machine.process(reinit);

	//Run
	state_machine.process(start);
	sleep(2);

	//Nothing
	state_machine.process(start);
	state_machine.process(reinit);

	//Init and back
	state_machine.process(stop);
	state_machine.process(start);
	sleep(2);

	//Emergency and back
	state_machine.process(emergency);
	state_machine.process(reinit);
	state_machine.process(start);
	sleep(2);

	//Pause
	state_machine.process(paused);

	//Nothing
	state_machine.process(start);
	state_machine.process(reinit);

	//Run and back
	state_machine.process(resume);
	state_machine.process(paused);
	sleep(2);

	//Init and back
	state_machine.process(stop);
	state_machine.process(start);
	sleep(2);
	state_machine.process(paused);

	//Emergency and back
	state_machine.process(emergency);
	state_machine.process(reinit);
	state_machine.process(start);
	sleep(2);
	state_machine.process(paused);

	//Emergency
	state_machine.process(emergency);

	//Nothing
	state_machine.process(start);
	state_machine.process(stop);
	state_machine.process(paused);
	state_machine.process(resume);
	state_machine.process(emergency);

	//Init
	state_machine.process(reinit);
}
