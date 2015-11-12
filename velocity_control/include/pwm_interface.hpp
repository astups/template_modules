/*****************************************************************************/
/*                        ASTUPS – libvelocity_control                       */
/*****************************************************************************/
/*
* Author : Julien Lechalupé - lechalupe [dot] julien [at] gmail [dot] com
*	Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
* Creation Date : 28/06/2015
* License : BSD-3-Clause
*/

/** \file
 * Inteface to all PWM controllers.
 */

#ifndef PWM_INTERFACE_HPP
#define PWM_INTERFACE_HPP

#include <pwm.hpp>

/** Possible type of PWM control.
 */
enum PwmType{servo, power, cytron};

/** Interface to all PWM implementation (servo and power).
 * To be implemented by the class derivating it.
 */
class PwmInterface{
protected:
	/** The actual PWM controlled. */
	Pwm _pwm;

	/** The current value sent to the PWM. */
	double _current_value;

	/** Forbidden constructor */
	PwmInterface();

public:
	/** Constructor to create the underlying PWM (with the period and pin).
	 * @param period Period for the PWM signal (in us)
	 * @param pin The pin to use, see PwmPin enumeration
	 */
	PwmInterface(unsigned long int period, PwmPin pin): _pwm(period, pin), _current_value(0) {}

	/** Set the value of the interface, must be a percentage in [0%, 100%].
	 * @param value Value to set
	 * @return Return false if invalid value or if can not apply it
	 */
	virtual bool setValue(double value) =0;

	/** Return the current value sent to the PWM.
	 * @return The current value
	 */
	double getValue(){
		return _current_value;
	}

	/** Empty destructor. */
	virtual ~PwmInterface() {};
};

#endif //PWM_INTERFACE_HPP
