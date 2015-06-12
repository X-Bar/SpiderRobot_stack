#include "ros/ros.h"
//~ #include "std_msgs/String.h"
#include "SpiderRobot_pkg/MyArray.h"

SpiderRobot_pkg::MyArray JointCommand;

void JointComCallback(const SpiderRobot_pkg::MyArray::ConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "/SpiderRobot/SpoofJointPublisher");
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/SpiderRobot/BaseJointCommand", 10, JointComCallback);
	//~ n.param<int>("/Outlet/MoveTimes", MoveTimes, 3);				// Times to move the base toward the plug
	ros::Rate loop_rate(5);
	
	JointCommand.speed = -1;											// start with invaild value to wait for real msg
	
	while (ros::ok() &&  n.ok())
	{
		if (JointCommand.speed >= 0)									// wait for a msg with a vaild speed command
		{
			switch (JointCommand.command)
			{
				case 0: // Update all joint commands via speed
				{
					
					break;
				}
				case 1: // Update all joint commands via time
				{
					
				}
				//~ case 9:
				default:
				{
					
				}
					
			}
		}
	} // end while (ros::ok() &&  n.ok())
	
	ros::spin();
	
	return 0;
}

void JointComCallback(const SpiderRobot_pkg::MyArray::ConstPtr& msg)
{
  ROS_INFO("I heard: ") ;
}
