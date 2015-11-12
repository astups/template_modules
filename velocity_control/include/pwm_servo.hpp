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
 * Handle PWM for servo.
 */

#ifndef PWM_SERVO_HPP
#define PWM_SERVO_HPP

#include <pwm_interface.hpp>

/** Class to control PWM for servo-motors.
 * The servo motors must be driven using a 20ms period, with an up time between 1ms and 2ms.
 */
class PwmServo: public PwmInterface{
public:
	/** Constructor.
	 * 20ms period.
	 * Set a default value of 1.5ms for up-time (middle position for servos).
	 * @param pin The pin to use, see PwmPin enumeration
	 */
	PwmServo(PwmPin pin): PwmInterface(20000, pin) {
		_pwm.setUpTime(1500);
	}

	/** Set the value of the interface, must be a percentage in [0%, 100%].
	 * N.B. Will actually be a percentage of [1ms, 2ms], not of the full period to allow proper control over servos
	 * @param value Value to set
	 * @return Return false if invalid value or if can not apply it
	 */
	bool setValue(double value){
		if(value<0. || value>100.){
			return false;
		}
		//Up time must be between 1ms and 2ms (or 1000us and 2000us)
		//So the convertion is up=1000*(value/100) + 1000
		if(!_pwm.setUpTime((1000*(value/100.))+1000.)){
			return false;
		}
		_current_value=value;
		return true;
	}

	/** Empty destructor. */
	~PwmServo() {}
};

#endif //PWM_SERVO_HPP
