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
 * Handle PWM for H-bridge cards.
 */

#ifndef PWM_POWER_HPP
#define PWM_POWER_HPP

#include <pwm_interface.hpp>

/** Class to control H-bridge cards (card that handles power).
 * The period for those PWM is high to prevent any earable noises.
 */
class PwmPower: public PwmInterface{
public:
	/** Constructor.
	 * 20us period (50kHz) to be above 22kHz (above earable).
	 * Set a default value of 0 for up-time.
	 * @param pin The pin to use, see PwmPin enumeration
	 */
	PwmPower(PwmPin pin): PwmInterface(20, pin) {
		_pwm.setPercent(0);
	}

	/** Set the value of the interface, must be a percentage in [0%, 100%].
	 * @param value Value to set
	 * @return Return false if invalid value or if can not apply it
	 */
	bool setValue(double value){
		if(value<0. || value>100.){
			return false;
		}
		if(!_pwm.setPercent(value)){
			return false;
		}
		_current_value=value;
		return true;
	}

	/** Empty destructor. */
	~PwmPower() {}
};

#endif //PWM_POWER_HPP
