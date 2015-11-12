/*****************************************************************************/
/*                  ASTUPS – velocity_control_state_machine                  */
/*****************************************************************************/
/*
* Author : Julien Lechalupé - lechalupe [dot] julien [at] gmail [dot] com
* Creation Date : 03/07/2015
* License : BSD-3-Clause
*/

/*
 *	Static rc car settings
 */
// For GPIO and PWM enum type
// Not cool to have all these includes
// TODO Maybe put all definition into one (pwm_interface ?) ?
#include <pwm.hpp>
#include <pwm_interface.hpp>
#include <pwm_cytron.hpp> 

const float max_speed = 1.0;	// (m/s)
const float max_angle = 30;		// (°)

const PwmPin speed_pin = PwmPin::GPIO_13;
const PwmPin angle_pin = PwmPin::GPIO_18;

const PwmType speed_type = PwmType::cytron;
const PwmType angle_type = PwmType::cytron;

const GpioPin speed_direction_pin = GpioPin::GPIO_15;
const GpioPin angle_direction_pin = GpioPin::GPIO_20;
