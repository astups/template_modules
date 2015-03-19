/*****************************************************************************/
/*                  ASTUPS – velocity_control_state_machine                  */
/*****************************************************************************/
/*
* Author : Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
* Creation Date : 07/02/2015
* License : BSD-3-Clause
*/

#include <state_machine.hpp>

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
PRINT_ONENTRY(Emergency);

STATE_ACTION(Run);
//EMPTY_ONENTRY(Run);
//EMPTY_ONEXIT(Run);
PRINT_ONENTRY(Run);
PRINT_ONEXIT(Run);

void Run::action(){
	// simulate expensive operation
	std::cout<<"  Start run loop"<<std::endl;
	
	bool quit;
	_quit_action_mutex.lock();
	quit=_quit_action;
	_quit_action_mutex.unlock();
	
	while(!quit){
		sleep(1);
		std::cout<<"  Run loop"<<std::endl;
		
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
