/***********************************************************************************************************************
FILENAME:   SpiderRobotMain_pub.cpp
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
6 legs, 3 DoF each. 18 servos, 18 channels
MyArray PosArray.data[18] makes angle positions for all 18 servos

Left side
Leg one: channels 0, 1, 2
Leg Two: channels 3, 4, 5
Leg Three: channels 6, 7, 8

Right side
Leg one: channels 9, 10, 11
Leg two: channels 12, 13, 14
Leg three: channels 15, 16, 17

Leg group 0
Leg one: channels 0, 1, 2
Leg two: channels 12, 13, 14
Leg Three: channels 6, 7, 8

Leg group 1
Leg one: channels 9, 10, 11
Leg Two: channels 3, 4, 5
Leg Three: channels 6, 7, 8

***********************************************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <iostream>

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include <SpiderRobot_pkg/MyArray.h>

//~ #include "SpiderRobot_pkg/AnglesToJoints.h"
//~ #include "SpiderRobot_pkg/InverseK.h"
//~ #include "SpiderRobot_pkg/TransferFrame.h"
#include "SpiderRobot_pkg/SpiderConstants.h"
#include "SpiderRobot_pkg/SpiderFunctions.h"

SpiderRobot_pkg::MyArray Angles2Joints(short int group, int Joints[3], SpiderRobot_pkg::MyArray PosArray);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);
short int WaitForDone(void);
float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void MultiplyMat(float A[4][4], float B[4][1], float C[4][1]);//,int N, int L, int M);
short int InverseKinematics(float BasePoints[3], int LegAng[3]);

using namespace SpiderRobotConstants;							// stores leg common leg positions

bool SHUTDOWN = false;											// flag to shutdown while loop
SpiderRobot_pkg::MyArray PosArray;									// ROS message to publish
bool CurrentlyMoving = true;									// flag of whether serial controller returns that its currenly moving
int STATE = 0;													// for state machine


int main(int argc, char **argv)
{
	printf("Starting SpiderRobotMain_pubsub \n");
	
	ros::init(argc, argv, "SpiderRobotMain");					// start ROS connectionn
	ros::NodeHandle nh;											// make node handle
	// make publishing object and advertise
	ros::Publisher SpiderRobotMain_pub = nh.advertise<SpiderRobot_pkg::MyArray>("MyArray", 100);
	// make subscribing object for feedback
	ros::Subscriber LegStatus_sub = nh.subscribe("LegStatus", 1, LegStatusCallback);

	int i = 0;													// counter
	short int MySpeed = 300;									// speed for joints to move
	float* pos;													// temp for holding calculated positions
	int LegAngs[3] = {0};										// temp for holding leg angles
	
	int* LegArray;
	PosArray.command = 0;										// command number. 0 is move all legs
	PosArray.speed = 300;										// speed for joints to move
	PosArray.size = 18;											// size of data array
	ros::Rate loop_rate(.2);									// while loop rate
	
	printf("Starting system loop..\n");
	STATE = 0; 													// set state to start
	
	/////////////
	
	//~ float pos1[] = {0.2032, 0.2032, -.15}; 
	//~ float* pos2;
	//~ pos2 = TransferFrame(0,0, pos1);
	//~ 
	//~ printf("\n\n pos: %f %f %f \n\n", pos2[0], pos2[1], pos2[2] );
	//~ 
	//~ int LegAngs[3] = {0};
	//~ InverseKinematics(LegUpup_Car_Leg, LegAngs);
	//~ 
	//~ PosArray = Angles2Joints(2, LegAngs, PosArray);
	//~ SpiderRobotMain_pub.publish(PosArray);				// publish command
	//~ usleep(1*1000*1000);
	//~ SpiderRobotMain_pub.publish(PosArray);		
	//~ usleep(3*1000*1000);
	//~ 
	//~ ////////////
	//~ InverseKinematics(LegUpOut_Car_Leg, LegAngs);
	//~ PosArray = Angles2Joints(2, LegAngs, PosArray);
	//~ SpiderRobotMain_pub.publish(PosArray);				// publish command
	//~ usleep(3*1000*1000);
	//~ 
	//~ InverseKinematics(LegDownOut_Car_Leg, LegAngs);
	//~ PosArray = Angles2Joints(2, LegAngs, PosArray);
	//~ SpiderRobotMain_pub.publish(PosArray);				// publish command
	//~ usleep(3*1000*1000);
	//~ 
	//~ InverseKinematics(LegUpOut_Car_Leg, LegAngs);
	//~ PosArray = Angles2Joints(2, LegAngs, PosArray);
	//~ SpiderRobotMain_pub.publish(PosArray);				// publish command
	//~ usleep(3*1000*1000);
	//~ 
	//~ InverseKinematics(LegDownOut_Car_Leg, LegAngs);
	//~ PosArray = Angles2Joints(2, LegAngs, PosArray);
	//~ SpiderRobotMain_pub.publish(PosArray);				// publish command
	//~ usleep(3*1000*1000);
	//~ 
	//~ return 0;
	/////////////
	
	while(ros::ok() && !SHUTDOWN)
	{
		switch(STATE)
		{
		  case 0: // start 
		  {
			//~ InverseKinematics(LegUpup_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegUpUp_Ang_Leg, PosArray);	// move all leg groups
			PosArray.speed = 100;										// do first move slowly
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			usleep(1*1000*1000);
			SpiderRobotMain_pub.publish(PosArray);				// publish first command twice
			CurrentlyMoving = true;								// change status to moving as command was given
			STATE = 1;
			PosArray.speed = 300;
			break;
	  	  }
		  case 1:
		  {
			InverseKinematics(LegUpOut_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);		// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 2;
			break;
		  }
		  case 2:
		  {
			InverseKinematics(LegDownOut_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);		// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 3;
			break;
		  }
		  case 3:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);		// move leg group 1
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 4;
			break;
		  }
		  case 4:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);		// move leg group 1
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 5;
			break;
		  }
		  case 5:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);		// move leg group 0
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 6;
			break;
		  }
		  case 6:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);		// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);		// move leg group 0
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			CurrentlyMoving = true;
			STATE = 3;
			break;
		  }
		  default:
		  {
			PosArray.command = 1;								// command 1 is exit for serial controller
			SpiderRobotMain_pub.publish(PosArray);				// publish command
			SHUTDOWN = true;									// stop while loop
			break; break;										// exit
		  }
		}// end switch(STATE)
		usleep(1000*1000);
		ros::spinOnce();
		ros::spinOnce();
		WaitForDone();											// wait for completetion
	}// end while(ros::ok() && !SHUTDOWN)





	printf("SHUTDOWN COMMAND DETECTED...\n\n");
}// end main

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DETECTED...\n");
	SHUTDOWN = true;
}

/***********************************************************************************************************************
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
* callback that recieves leg status and set globle bool variable 
* CurrentlyMoving so that other functions know
***********************************************************************************************************************/
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	//~ printf("Leg Status feedback: %c\n", msg->data);
	switch(msg->data)
	{
	  case '.': // Not moving
	  {
		CurrentlyMoving = false;
		break;
	  }
	  case '+': // Moving
	  {
		CurrentlyMoving = true;
		break;
	  }
	}
}// end LegStatusCallback()

/***********************************************************************************************************************
short int WaitForDone(void)
* forces code wait till function LegStatusCallback() declares robot
* is ready to move
***********************************************************************************************************************/
short int WaitForDone(void)
{
	//for(int i = 0; i < 1000; i++)
	while(CurrentlyMoving && ros::ok() && !SHUTDOWN)
	{
		ros::spinOnce();										// Check for leg status msg
		usleep(100*1000);										// wait 10th of a second
	}
	return 0;
}// end WaitForDone()




