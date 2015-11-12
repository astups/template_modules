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
 * Handle PWM to control steering.
 */

#ifndef ANGLE_COMMAND_HPP
#define ANGLE_COMMAND_HPP

#include <stdexcept>

#include <pwm_interface.hpp>
#include <pwm_servo.hpp>
#include <pwm_power.hpp>
#include <pwm_cytron.hpp>

/** Class to control the steering of a car (the direction).
 * Controls it using a PWM
 */
class AngleCommand{
private:
	/** Hides the actual implementation of the PWM (can be a PWM for servo or for power circuit) */
	PwmInterface* _pwm;

	/** The mechanical/physical maximum angle the car can have, is just used to limit the input commands. */
	double _maximum_angle;

	/** The current angle command (not the actual angle, may be different if under constraints). */
	double _current_angle;

	/** Polarity of the angle, allow to change the direction of control without rewiring. */
	bool _polarity;

	/** Forbidden constructor. */
	AngleCommand();

public:
	/** Constructor.
	 * @param maximum_angle The maximum admissible angle for the car (the physical maximum)
	 * @param type The type of control hardware can be a servo or a power circuit
	 * @param pin_number The pin to use, see PwmPin enumeration
	 * @param direction_pin The direction pin, used only if using PWM controlling cytron (see PwmCytron)
	 */
	AngleCommand(double maximum_angle, PwmType type, PwmPin pin_number, GpioPin direction_pin=unknown):
	_pwm(NULL), _maximum_angle(maximum_angle), _current_angle(0), _polarity(false){
		if(_maximum_angle>90. || _maximum_angle<0){
			throw(std::invalid_argument("Maximum angle must be between 0 and 90 degrees"));
		}

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

	/** Set the command angle.
	 * @param angle Command angle, must be in [-_maximum_angle, _maximum_angle]
	 * @return False if the angle is out of bound or not applicable
	 */
	bool setAngle(double angle){
		if(angle<-_maximum_angle || angle>_maximum_angle){
			return false;
		}
		angle=(_polarity?angle:-angle);
		if(!_pwm->setValue((angle*100.)/(2*_maximum_angle)+50.)){
			return false;
		}
		_current_angle=angle;
		return true;
	}

	/** Set the command angle as an percent of the maximum angle.
	 * @param value Command angle, must be in [-100%, 100%], e.g. 100%=_maximum_angle
	 * @return False if the angle is out of bound or not applicable
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
		_current_angle=value*_maximum_angle;
		return true;
	}

	/** Get the current angle command (not the actual angle, may be different if under constraints).
	 * @return The current angle (in degrees)
	 */
	double getAngle(){
		return _current_angle;
	}

	/** Get the current angle command as a percentage (not the actual angle, may be different if under constraints).
	 * @return The current angle (in percent of the maximum angle)
	 */
	double getValue(){
		return _current_angle/_maximum_angle;
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
	~AngleCommand(){
		delete(_pwm);
	}
};

#endif //ANGLE_COMMAND_HPP
