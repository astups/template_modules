/*****************************************************************************/
/*                        ASTUPS – libvelocity_control                       */
/*****************************************************************************/
/*
* Author : Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
*	Julien Lechalupé - lechalupe [dot] julien [at] gmail [dot] com
* Creation Date : 28/06/2015
* License : BSD-3-Clause
*/

/** \file
 * Handle PWM to control speed.
 */

#ifndef SPEED_COMMAND_HPP
#define SPEED_COMMAND_HPP

#include <stdexcept>

#include <pwm_interface.hpp>
#include <pwm_servo.hpp>
#include <pwm_power.hpp>
#include <pwm_cytron.hpp>

/** Class to control the speed of a car.
 * Controls it using a PWM
 */
class SpeedCommand{
private:
	/** Hides the actual implementation of the PWM (can be a PWM for servo or for power circuit) */
	PwmInterface* _pwm;

	/** The mechanical/physical maximum speed the car can have, is just used to limit the input commands. */
	double _maximum_speed;

	/** The current speed command (not the actual speed, may be different if under constraints like drifting). */
	double _current_speed;

	/** Polarity of the angle, allow to change the direction of control without rewiring. */
	bool _polarity;

	/** Forbidden constructor. */
	SpeedCommand();

public:
	/** Constructor.
	 * @param maximum_speed The maximum admissible speeed for the car (the physical maximum)
	 * @param type The type of control hardware can be a servo or a power circuit
	 * @param pin_number The pin to use, see PwmPin enumeration
	 * @param direction_pin The direction pin, used only if using PWM controlling cytron (see PwmCytron)
	 */
	SpeedCommand(double maximum_speed, PwmType type, PwmPin pin_number, GpioPin direction_pin=unknown):
	_pwm(NULL), _maximum_speed(maximum_speed), _current_speed(0), _polarity(false){
		switch(type){
			case servo: _pwm=new PwmServo(pin_number);
				if(direction_pin!=-1) std::cout<<"WARNING: Giving a direction pin for a servo pwm, will not be used"<<std::endl;
				break;
			case power: _pwm=new PwmPower(pin_number);
				if(direction_pin!=-1) std::cout<<"WARNING: Giving a direction pin for a power pwm, will not be used"<<std::endl;
				break;
			case cytron: if(direction_pin==-1)
					throw std::invalid_argument("ERROR: Not giving a direction pin for a cytron pwm, while it is mandatory.");
				_pwm=new PwmCytron(pin_number, direction_pin);
				break;
		}
	}

	/** Set the command speed.
	 * @param speed Command speed, must be in [-_maximum_speed, _maximum_speed]
	 * @return False if the speed is out of bound or not applicable
	 */
	bool setSpeed(double speed){
		if(speed<-_maximum_speed || speed>_maximum_speed){
			return false;
		}
		speed=(_polarity?speed:-speed);
		if(!_pwm->setValue((speed*100.)/(2*_maximum_speed)+50.)){
			return false;
		}
		_current_speed=speed;
		return true;
	}

	/** Set the command speed as an percent of the maximum speed.
	 * @param value Command speed, must be in [-100%, 100%], e.g. 100%=_maximum_speed
	 * @return False if the speed is out of bound or not applicable
	 */
	bool setValue(double value){
		if(value<-100 || value>100){
			return false;
		}
		value=(_polarity?value:-value);
		//To get the value: output = value/2 + 50 = (value + 100)/2
		//Changes the range from [-100, 100] to [0, 100]
		if(!_pwm->setValue((value+100)/2)){
			return false;
		}
		_current_speed=value*_maximum_speed;
		return true;
	}

	/** Get the current speed command (not the actual speed, may be different if under constraints like drifting).
	 * @return The current speed (in m/s)
	 */
	double getSpeed(){
		return _current_speed;
	}

	/** Get the current speed command as a percentage (not the actual speed, may be different if under constraints like drifting).
	 * @return The current speed (in percent of maximum speed)
	 */
	double getValue(){
		return _current_speed/_maximum_speed;
	}

	/** Set the polarity, allows to change the direction of the steering wheel (without needing to rewire the robot).
	 * @param polarity New polarity (default value = false)
	 */
	void setPolarity(bool polarity){
		_polarity=polarity;
	}

	/** Get the polarity.
	 * See setPolarity.
	 * @return Current polarity
	 */
	bool getPolarity(){
		return _polarity;
	}

	/** Destructor.
	 */
	~SpeedCommand(){
		delete(_pwm);
	}
};

#endif //SPEED_COMMAND_HPP
