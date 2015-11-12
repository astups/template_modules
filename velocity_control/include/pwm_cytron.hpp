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
 * Handle PWM for Cytron boards (H-bridge).
 */

#ifndef PWM_CYTRON_HPP
#define PWM_CYTRON_HPP

#include <pwm_interface.hpp>

/*! \enum GpioPin 
 * List of possible GPIO pins to use for direction pin.
 */
enum GpioPin{unknown=-1, GPIO_2=2, GPIO_3=3, GPIO_4=4, GPIO_5=5, GPIO_6=6, GPIO_7=7, GPIO_8=8, GPIO_9=9, GPIO_10=10, GPIO_11=11, GPIO_12=12, GPIO_14=14, GPIO_15=15, GPIO_17=17, GPIO_16=16, GPIO_19=19, GPIO_20=20, GPIO_21=21, GPIO_22=22, GPIO_23=23, GPIO_24=24, GPIO_25=25, GPIO_26=26, GPIO_27=27}; //Removed GPIO_13 and GPIO_18 (see pwm.hpp)

/** Class to control Cytron H-bridge cards (card that handles power).
 * The period for those PWM has to be 20ms and an additionnal pin is needed to
 * control the direction.
 */
class PwmCytron: public PwmInterface{
private:
	/** The pin used to change the direction of the output of the cytron board. */
	GpioPin _direction_pin;

public:
	/** Constructor.
	 * 20ms period.
	 * Set a default value of 0 for up-time.
	 * @param pin The pin to use, see PwmPin enumeration
	 * @param direction_pin The pin used to change the direction of the output
	 */
	PwmCytron(PwmPin pin, GpioPin direction_pin): PwmInterface(20000, pin), _direction_pin(direction_pin) {
		_pwm.setPercent(0);

		if(_direction_pin==unknown){
			throw std::invalid_argument("ERROR: Not giving a direction pin for a cytron pwm, while it is mandatory.");
		}

		//Set GPIO pin mode
		if(wiringPiSetupGpio() == -1)
		{
			throw(std::runtime_error("Error: wiringPi failed to initialize"));
		}
		pinMode(_direction_pin, OUTPUT);
	}

	/** Set the value of the interface, must be a percentage in [0%, 100%].
	 * WARNING: The range is understand as for servos: in [0%, 50%] it uses a direction (0% corresponding to the maximum command)
	 * while [50%, 100%] corresponds to the other direction.
	 * @param value Value to set
	 * @return Return false if invalid value or if can not apply it
	 * \TODO Fix the test value==50 (instead use a threashold test)
	 */
	bool setValue(double value){
		if(value<0. || value>100.){
			return false;
		}

		//If the value is exactly 50 do not send command (the direction does not matter)
		if(value==50.){
			if(!_pwm.setPercent(0)){
				return false;
			}
		}

		//Transform from [0%, 100%] to [-100%, 100%], where the sign will give the direction (remove it to get the actual value)
		double temp_value=(2.*value)-100;

		bool direction=(temp_value>0);
		double real_value=fabs(temp_value);

		//Set the direction
		digitalWrite(_direction_pin, (direction?HIGH:LOW));

		//Finally set the value
		if(!_pwm.setPercent(real_value)){
			return false;
		}
		_current_value=value;
		return true;
	}

	/** Empty destructor. */
	~PwmCytron() {}
};

#endif //PWM_CYTRON_HPP
