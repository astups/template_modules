

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ros_subscriber.hpp>
#include <ros_publisher.hpp>
#include <velocity_control/command.h>
#include <velocity_control/event.h>

#include <velocity_control_state_machine.hpp>

void commandCallback(const boost::shared_ptr<velocity_control::command>& data)
{
	if(data==NULL){
		ROS_ERROR("Received invalid message");
		return;
	}

	ROS_INFO(" [Receive] <%f, %f>\t-\t%fs\t-\t%s", data->linear, data->angular, data->duration, (data->override?"override":"no-override"));
	Run.pushAction((*data));
}

void eventCallback(const boost::shared_ptr<velocity_control::event>& data)
{
	if(data==NULL){
		ROS_ERROR("Received invalid message");
		return;
	}

	std::string event=data->event;

	ROS_INFO(" [Receive] event:%s", event.c_str());
	
	
	if(event=="pause")
	{
		state_machine.process(paused);
	}
	else if(event=="resume")
	{
		state_machine.process(resume);
	} 
	else if(event=="stop")
	{
		state_machine.process(stop);
	} 
	else if(event=="emergency")
	{
		state_machine.process(emergency);
	} 
	else if(event=="start")
	{
		state_machine.process(start);
	}
	else if(event=="reinit")
	{
		state_machine.process(reinit);
	} 
	else 
	{
		ROS_ERROR("Wrong event message: %s", event.c_str());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"velocity_control");

//Default way to do
//	ros::NodeHandle n("~");

//	ros::Subscriber sub = n.subscribe("velocity_command", 64, &commandCallback);
//	ros::Subscriber sub2 = n.subscribe("velocity_event", 64, &eventCallback);

//Our way to do
	RosSubscriber<velocity_control::command> subCommand("velocity_command",64,&commandCallback);
	RosSubscriber<velocity_control::event> subEvent("velocity_event",64,&eventCallback);

//Parse the type of robot to configure the correct PWM and the limits (linear speed, ...)
//	std::string robotType;
//	
//	if (n.getParam("robot", robotType)) {
//		ROS_INFO("Got parameter: %s", robotType.c_str());
//	} else {
//		ROS_ERROR("Failed to get param 'robot'");
//	}

	while(ros::ok())
	{
		ros::spinOnce();
		
		// 25ms sleep to give CU some rest
		usleep(25000);
	}
	
	return 0;
}
