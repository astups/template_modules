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
#include <config.hpp>

#include <velocity_control/command.h>
#include <velocity_control/event.h>
#include <ros_publisher.hpp>
#include <string>
#include <iostream>

#define WAITKEYPRESS 	ROS_INFO("\t<Press any key to continue>\n\n"); \
						std::cin.get

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"velocity_control_tester");

	// Create communication objects
	RosPublisher<velocity_control::command> pubCommand("/velocity_control/velocity_command",64);
	RosPublisher<velocity_control::event> pubEvent("/velocity_control/velocity_event",64);

	// Create test variables
	velocity_control::command 	commandData;
	velocity_control::event 	eventData;

	int mode;

	if ( argc < 2 )
	{
		std::cout 	<< "usage: velocity_control_tester <mode>" << std::endl
					<< "<mode> : S (s trajectory) | T (unit tests)" << std::endl;
		return 0;
	}
	else
	{
		if ( std::string(argv[1]) == "S" )
		{
			mode = 1;
		}
		else if ( std::string(argv[1]) == "T" )
		{
			mode = 2;
		}
		else
		{
			std::cout << "Wrong mode !" << std::endl;
			std::cout 	<< "usage: velocity_control_tester <mode>" << std::endl
						<< "<mode> : S (s trajectory) | T (unit tests)" << std::endl;
			return 0;
		}
	}

	usleep(500000);


	if ( mode == 1)
	{
		eventData.event = "start";
		pubEvent.publish(eventData);
		ROS_INFO("Send START event...");
		usleep(1000000);

		ROS_INFO("Send commands...");

		// Full forward 5%
		commandData.linear 		= 0.05;
		commandData.angular 	= 0.0;
		commandData.duration 	= 3.0;
		commandData.override	= false;
		pubCommand.publish(commandData);

		// Full-left 5%
		commandData.linear 		= 0.05;
		commandData.angular 	= -max_angle;
		commandData.duration 	= 3.0;
		commandData.override	= false;
		pubCommand.publish(commandData);

		// Full-right 5%
		commandData.linear 		= 0.05;
		commandData.angular 	= max_angle;
		commandData.duration 	= 3.0;
		commandData.override	= false;
		pubCommand.publish(commandData);

		// Full forward 5%
		commandData.linear 		= 0.05;
		commandData.angular 	= 0.0;
		commandData.duration 	= 3.0;
		commandData.override	= false;
		pubCommand.publish(commandData);

		ROS_INFO("Done !");
	}
	else if ( mode == 2 )
	{
		eventData.event = "start";
		pubEvent.publish(eventData);
		ROS_INFO("Send START event...");
		usleep(1000000);

		ROS_INFO("Send commands...");
		// All test performed here are specified in velocity_control.odt
		// Please refer to that document for more informations about
		// the tests
		///////////////////////////////////////////////////////////////////
		// First pass: simple commands
		ROS_INFO(" First pass\n---------------------------");
		// Test 1
		commandData.linear 		= 0.2;
		commandData.angular 	= 0.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 0.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 2
		commandData.linear 		= -0.2;
		commandData.angular 	= 0.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 0.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 3
		commandData.linear 		= 0.0;
		commandData.angular 	= 20.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.0 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved wheels.");
		WAITKEYPRESS();

		// Test 4
		commandData.linear 		= 0.2;
		commandData.angular 	= 20.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 5
		commandData.linear 		= 0.2;
		commandData.angular 	= -20.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 6
		commandData.linear 		= -0.2;
		commandData.angular 	= 20.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 7
		commandData.linear 		= -0.2;
		commandData.angular 	= -20.0;
		commandData.duration 	= 1.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		///////////////////////////////////////////////////////////////////
		// Second pass: simple commands with long durations
		ROS_INFO(" First pass\n---------------------------");
		// Test 1
		commandData.linear 		= 0.2;
		commandData.angular 	= 0.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 0.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 2
		commandData.linear 		= -0.2;
		commandData.angular 	= 0.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 0.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 3
		commandData.linear 		= 0.0;
		commandData.angular 	= 20.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.0 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved wheels.");
		WAITKEYPRESS();

		// Test 4
		commandData.linear 		= 0.2;
		commandData.angular 	= 20.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 5
		commandData.linear 		= 0.2;
		commandData.angular 	= -20.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] 0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 6
		commandData.linear 		= -0.2;
		commandData.angular 	= 20.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();

		// Test 7
		commandData.linear 		= -0.2;
		commandData.angular 	= -20.0;
		commandData.duration 	= 5.0;
		commandData.override	= false;

		ROS_INFO(" [Send] -0.2 ms - 20.0 deg - 1.0 sec - no override");
		pubCommand.publish(commandData);
		ROS_INFO(" Test send please verify that the vehicle as moved.");
		WAITKEYPRESS();
	}

	return 0;
}
