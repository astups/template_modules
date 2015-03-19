/*****************************************************************************/
/*                            ASTUPS – state_machine                         */
/*****************************************************************************/
/*
* Author : Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
* Creation Date : 07/02/2015
* License : BSD-3-Clause
*/

#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

#include <iostream>
#include <thread>
#include <mutex>
#include <map>

enum event: unsigned int;

class State{
public:
	virtual void onEntry(event e)=0;
	virtual void onExit(event e)=0;

	virtual bool hasAction() {return false;}
	virtual void startAction() {}
	virtual void stopAction() {}
};

class StateMachine{
private:
	static const std::map<State*, std::map<event, State*>> _transition_table;
	static State* _current_state;

public:
	StateMachine() {}

	//Process an event, advance the state apply onExit of old state and onEntry on the new state
	bool process(event to_process){
		std::map<State*, std::map<event, State*>>::const_iterator transition;
		transition=_transition_table.find(_current_state);
		if(transition!=_transition_table.end()){
			std::map<event, State*>::const_iterator new_state;
			new_state=transition->second.find(to_process);

			if(new_state!=transition->second.end()){
				if(new_state->second==NULL){
					std::cerr<<"ERROR: Target state is NULL"<<std::endl;
					return false;
				}
				if(_current_state->hasAction()) _current_state->stopAction();
				_current_state->onExit(to_process);
				_current_state=new_state->second;
				_current_state->onEntry(to_process);
				if(_current_state->hasAction()) _current_state->startAction();
			} //else stay in state
		}else{
			std::cerr<<"ERROR: Invalid current state"<<std::endl;
			return false;
		}
		return true;
	}
} state_machine;

//Set of define to ease the definition of a new state machine
#define STATE(name) \
class name: public State{ \
	void onEntry(event e); \
	void onExit(event e); \
} name
#define STATE_ACTION(name) \
class name: public State{ \
public: \
	std::thread* _action_on_state; \
	bool _quit_action; \
	std::mutex _quit_action_mutex; \
	void action(); \
	void onEntry(event e); \
	void onExit(event e); \
	bool hasAction() {return true;} \
	void startAction(){ \
		_quit_action_mutex.lock(); \
		_quit_action=false; \
		_quit_action_mutex.unlock(); \
		_action_on_state=new std::thread(&Run::action, this); \
	} \
	void stopAction(){ \
		_quit_action_mutex.lock(); \
		_quit_action=true; \
		_quit_action_mutex.unlock(); \
		if(_action_on_state && std::this_thread::get_id()!=_action_on_state->get_id()) \
			_action_on_state->join(); \
	} \
} name;





#define EMPTY_ONENTRY(state) \
	void state::onEntry(event e) {}
#define PRINT_ONENTRY(state) \
	void state::onEntry(event e) { \
		std::cout<<"Entry " #state<<std::endl; \
	}





#define EMPTY_ONEXIT(state) \
	void state::onExit(event e) {}
#define PRINT_ONEXIT(state) \
	void state::onExit(event e) { \
		std::cout<<"Exit " #state<<std::endl; \
	}





#define INIT_STATE(name) \
	State* StateMachine::_current_state(&name)





//Redefine the sleep to prevent the include of unistd.h
#define sleep(s) \
	std::this_thread::sleep_for(std::chrono::seconds(s));
#endif // STATE_MACHINE_HPP_
