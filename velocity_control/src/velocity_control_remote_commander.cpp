/*****************************************************************************/
/*                  ASTUPS – velocity_control_state_machine                  */
/*****************************************************************************/
/*
* Author : Julien LECHALUPÉ - lechalupe [dot] julien [at] gmail [dot] com
* Creation Date : 11/04/2015
* License : BSD-3-Clause
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <velocity_control/command.h>
#include <velocity_control/event.h>
#include <ros_publisher.hpp>
#include <iostream>

#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#include <config.h>

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"velocity_control_tester");

	// Create communication objects
	RosPublisher<velocity_control::command> pubCommand("/velocity_control/velocity_command",64);

	// Create command variable
	velocity_control::command 	commandData;

	// Axis sensivity (thresold before movement [0; 100])
	const float sensivity = 8;
	// Axis position
	float x_axis;
	float y_axis;

	usleep(500000);

	// Check if a gamepad is connected
	if ( !sf::Joystick::isConnected(0) )
	{
		std::cout << "No gamepad detected, please connect one." << std::endl;
		return 0;
	}

	// If it exists, check if there is X and Y axis
	if ( !sf::Joystick::hasAxis(0,sf::Joystick::X) || !sf::Joystick::hasAxis(0,sf::Joystick::Y) )
	{
		std::cout << "No X/Y axis found, please check your gamepad axis" << std::endl;
		return 0;
	}

	// Do until comes death !
	while ( ros::ok() )
	{
		// Read Joystick commands
		sf::Joystick::update();
		x_axis = sf::Joystick::getAxisPosition(0,sf::Joystick::X);
		y_axis = sf::Joystick::getAxisPosition(0,sf::Joystick::Y);

		// Interpret it
		if ( x_axis > sensivity || x_axis < -sensivity )
		{
			commandData.linear = x_axis/100.0*maxSpeed;
		}
		else
		{
			commandData.linear = 0.0;
		}

		if ( y_axis > sensivity || y_axis < -sensivity )
		{
			commandData.angular = y_axis/100.0*maxAngle;
		}
		else
		{
			commandData.angular = 0.0;
		}
		
		commandData.duration 	= 0.05;
		commandData.override	= false;

		// Send comment and log
		ROS_INFO(" [Send] %f ms - %f deg - %f sec - %s",commandData.linear, commandData.angular,
				 commandData.duration, commandData.override?"override":"no override");

		pubCommand.publish(commandData);

		usleep(49500);
	}

	return 0;
}
