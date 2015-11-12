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
 * Wrapper to wiringPi PWM.
 */

#ifndef PWM_HPP
#define PWM_HPP

/** Defines the clock speed 19.2MHz.
 * \TODO See if it is possible to use CLOCK_BASE from wiringPi instead
 */
#define BASE_CLOCK 19200000 //Hz <=> 19.2MHz

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <wiringPi/wiringPi.h>

/** List of possible GPIO pins for the PWM.
 * \TODO Check for GPIO_19 if it works
 */
enum PwmPin {GPIO_13=13, GPIO_18=18};

/** Allow easy use of hardware PWMs (on Raspberry Pi B+).
 * Low-level class offering a wrapper to the wiring Pi library.
 */
class Pwm
{
	private:
		/** Forbid to instanciate Pwm without parameters */
		Pwm();

		/** Pin number */
		PwmPin _pin;
		/** Period (in us) */
		unsigned long int _period;
		/** UpTime (in us) */
		unsigned long int _up_time;

		/** The divisor represents the precision of the PWM */
		static constexpr double _divisor = 1024.;

	public:
		/** Constructs a PWM.
		 * Create the PWM and calls the wiringPi library to start it.
		 * Warning: it does not set a default value.
		 * @param period PWM period in us
		 * @param pin Pin number to use (see PwmPin enum to know all possible values)
		 */
		Pwm(unsigned long int period, PwmPin pin): _pin(pin), _period(period), _up_time(0)
		{
			if(wiringPiSetupGpio() == -1)
			{
				throw(std::runtime_error("Error: wiringPi failed to initialize"));
			}

			pinMode(_pin, PWM_OUTPUT);
			pwmSetMode(PWM_MODE_MS);

			// Calculate divisor for the desired period
			double freq = 1000000.0/(double)(_period);

			pwmSetClock(floor(BASE_CLOCK/(freq*_divisor)));
			pwmSetRange((int)_divisor);
		}

		/** Set the up-time for the PWM.
		 * @param up_time Up-time in us
		 * @return True if everything is ok
		 */
		bool setUpTime(unsigned long int up_time) //us
		{
			if ( round((_divisor*up_time)/_period) > _divisor )
			{
				return false;
			}

			_up_time = up_time;

			pwmWrite(_pin,(_divisor*_up_time)/_period);

			return true;
		}

		/** Set the duty-cycle for the PWM.
		 * @param percent Duty-cycle (in %)
		 * @return True if everything is ok
		 */
		bool setPercent(double percent) //%
		{
			if ( percent < 0.0 || percent > 100.0 )
			{
				return false;
			}

			_up_time = percent/100.0*_period;

			pwmWrite(_pin,(_divisor*_up_time)/_period);

			return true;
		}

		/** Get the period of the PWM.
		 * @return The period in us
		 */
		unsigned long int getPeriod() const
		{
			return _period;
		}

		/** Get the current up-time.
		 * (Is updated even when using the setPercent.)
		 * @return The current up-time in us
		 */
		unsigned long int getUpTime() const
		{
			return _up_time;
		}

		/** Get the current duty-cycle.
		 * (Is updated even when using the setUpTime.)
		 * @return The current duty-cycle in %
		 */
		double getPercent() const
		{
			return _up_time/(double)(_period)*100.0;
		}

};

#endif //PWM_HPP
