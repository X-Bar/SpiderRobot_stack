/***********************************************************************************************************************
FILENAME:   SpiderRobot.cpp
AUTHORS:    Cody L. Lundberg

DESCRIPTION:
Planning node for SpiderRobot package

PUBLISHES:  "MyArray" SpiderRobot/MyArray
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
05.20.2013   CDM     Cody L. Lundberg original file creation

***********************************************************************************************************************
SpiderRobot Legs
6 legs, 3 DoF each. 18 servos
MyArray PosArray.data[18] makes angle positions for all 18 servos

Left side
Leg one: channels 0, 1, 2
Leg Two: channels 3, 4, 5
Leg Three: channels 6, 7, 8

Right side
Leg one: channels 9, 10, 11
Leg two: channels 12, 13, 14
Leg three: channels 15, 16, 17

***********************************************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Char.h"
//#include <geometry_msgs/TwistStamped.h>
#include <iostream>

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include <SpiderRobot/MyArray.h>

void Angles2Joints(short int group, int Joints[3]);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);

bool SHUTDOWN = false;								// flag to shutdown while loop
SpiderRobot::MyArray PosArray;							// ROS message to publish
bool CurrentlyMoving = true;							// flag of whether serial controller returns that its currenly moving

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SpiderRobotMain");				// start ROS connection
	ros::NodeHandle nh;							// make node handle
	// make publishing object and advertise
	ros::Publisher SpiderRobotMain_pub = nh.advertise<SpiderRobot::MyArray>("MyArray", 100);
	// make subscribing object for feedback
	ros::Subscriber LegStatus_sub = nh.subscribe("LegStatus", 1, LegStatusCallback);

	int i = 0;								// counter
	short int MySpeed = 300;						// speed for joints to move
	
	int* LegArray;
	PosArray.command = 0;							// command number. 0 is move all legs
	PosArray.speed = 300;							// speed for joints to move
	PosArray.size = 18;							// size of data array
	ros::Rate loop_rate(.2);						// while loop rate
	
	// number of general positions, old version
	//  side/leg/joint     L/1/1      L/1/3   L/2/2      L/3/1     L/3/3     R/1/2     R/2/1     R/2/3     R/3/2                      
	//  side/leg/joint          L/1/2     L/2/1     L/2/3     L/3/2     R/1/1     R/1/3     R/2/2     R/3/1    R/3/3                     
	//  channel      Lside	 0    1    2    3    4    5    6    7     8  RS9  10   11   12    13  14    15  16    17
	int AllLegsUpUp[] = 	{0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80};
	int AllLegsHome[] = 	{0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0};
	int LegG0Down[] = 	{0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0};
	int LegG0Up[] = 	{0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0};
	int LegG1Down[] = 	{0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15};
	int LegG1Up[] = 	{0,   0,    0,  0,  27,   27,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25};

	// number of general positions, new version. Use with function Angles to Joints
	//			J0  J1  J2
	int LegsUpUp[] =	{0, 63, -80};					// position for leg group all straight up
	int LegsHome[] =	{0, 0, 0};					// leg all at zero
	int LegsDown[] =	{0, -30, -35};					// legs in a standing down position
	int LegsUp[] =		{0, 27, 25};					// legs in a lefted position

	printf("Starting...\n");
	





	printf("SHUTDOWN COMMAND DETECTED...\n\n");
}// end main

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DETECTED...\n");
	SHUTDOWN = true;
}

void Angles2Joints(short int group, int Joints[3])
{
	switch(group)
	{
	 case 0: // leg group 0
	 {
		PosArray.data[0] = Joints[0];
		PosArray.data[1] = Joints[1];
		PosArray.data[2] = Joints[2];
		PosArray.data[6] = Joints[0];
		PosArray.data[7] = Joints[1];
		PosArray.data[8] = Joints[2];
		PosArray.data[12] = Joints[0];
		PosArray.data[13] = Joints[1];
		PosArray.data[14] = Joints[2];
		break;
	 }
	 case 1: // leg group 1
	 {
		PosArray.data[3] = Joints[0];
		PosArray.data[4] = Joints[1];
		PosArray.data[5] = Joints[2];
		PosArray.data[9] = Joints[0];
		PosArray.data[10] = Joints[1];
		PosArray.data[11] = Joints[2];
		PosArray.data[15] = Joints[0];
		PosArray.data[16] = Joints[1];
		PosArray.data[17] = Joints[2];
		break;
	 } 
	 case 2: // all legs
	 {
		PosArray.data[0] = Joints[0];
		PosArray.data[1] = Joints[1];
		PosArray.data[2] = Joints[2];
		PosArray.data[6] = Joints[0];
		PosArray.data[7] = Joints[1];
		PosArray.data[8] = Joints[2];
		PosArray.data[12] = Joints[0];
		PosArray.data[13] = Joints[1];
		PosArray.data[14] = Joints[2];

		PosArray.data[3] = Joints[0];
		PosArray.data[4] = Joints[1];
		PosArray.data[5] = Joints[2];
		PosArray.data[9] = Joints[0];
		PosArray.data[10] = Joints[1];
		PosArray.data[11] = Joints[2];
		PosArray.data[15] = Joints[0];
		PosArray.data[16] = Joints[1];
		PosArray.data[17] = Joints[2];
		break;
	 }// end case 3
	}// end switch
	
}// end Angles2Joints

void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	printf("Leg Status feedback: %c\n", msg->data);
}


