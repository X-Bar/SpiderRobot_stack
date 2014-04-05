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

**********************************************************************************************************************/

#include <stdio.h>
#include <signal.h>														// for ctrl c exit
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/Twist.h"
#include <SpiderRobot_pkg/MyArray.h>									// to publish joint angles
#include "ros/callback_queue.h"											// for custom callbacks
#include <boost/thread.hpp>

#include "SpiderRobot_pkg/SpiderConstants.h"
#include "SpiderRobot_pkg/SpiderFunctions.h"

void callbackThread(void);
SpiderRobot_pkg::MyArray Angles2Joints(short int group, int Joints[3], SpiderRobot_pkg::MyArray PosArray);
float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg);
void shutdownHandler(int s);
short int WaitForDone(void);

using namespace SpiderRobotConstants;									// stores leg common leg positions

ros::CallbackQueue g_queue;												// custom queue for leg status feedback
boost::thread RobotTwist_thread(callbackThread);

bool CurrentlyMoving = true;											// flag of whether serial controller returns that its currenly moving
int STATE = 0;															// for state machine
bool SHUTDOWN = false;													// flag to shutdown while loop
short int LoopHz = 10;													// rate for loops
SpiderRobot_pkg::MyArray PosArray;										// ROS message to publish
short int LegGroupTurn = 0;											// leg group 0 or 1's turn to move
float Zposition = 0;													// will eventually modify standing height
float Xstride = 0;														// walking stride in X, Y, Z, and rotation theta
float Ystride = 0;														
float Zstride = .00;													// amount legs left up while walking, default 1cm
float Tstride = 0;														
float TwistFactor = 1000*1000;											// convert between raw twist and meter

float FPG0L0_C[3];														// holds foot plant points in 
float FPG0L1_C[3];														
float FPG0L2_C[3];														// cartesian coordinates
float FPG1L0_C[3];
float FPG1L1_C[3];
float FPG1L2_C[3];
int FPG0L0_A[3];														// holds foot plant points in 
int FPG0L1_A[3];														
int FPG0L2_A[3];														// angle coordinates
int FPG1L0_A[3];
int FPG1L1_A[3];
int FPG1L2_A[3];
//~ int *FPG0L0_A;														// holds foot plant points in 
//~ int *FPG0L1_A;														
//~ int *FPG0L2_A;														// angle coordinates
//~ int *FPG1L0_A;
//~ int *FPG1L1_A;
//~ int *FPG1L2_A;



//~ float *FPG0L0;														// holds foot plant points in 
//~ float *FPG0L1;														
//~ float *FPG0L2;														// cartesian coordinates
//~ float *FPG1L0;
//~ float *FPG1L1;
//~ float *FPG1L2;

// make publishing object and advertise. make globe so callback can publish
ros::Publisher SpiderRobotMain_pub;// = nh.advertise<SpiderRobot_pkg::MyArray>("MyArray", 100);

int main(int argc, char **argv)
{
	ROS_INFO("Starting SpiderMovementController_pubsub \n");
	
	ros::init(argc, argv, "MovementController");						// start ROS connection
	ros::NodeHandle nh;													// make node handle
	
	// make publishing object and advertise
	SpiderRobotMain_pub = nh.advertise<SpiderRobot_pkg::MyArray>("MyArray", 100);
	
	// make subscribing object for feedback
	ros::Subscriber LegStatus_sub = nh.subscribe("LegStatus", 1, LegStatusCallback);
	//~ ros::Subscriber RobotTwist_sub = nh.subscribe("RobotTwist", 1, RobotTwistCallback); // moved to after while(ros::ok() && !SHUTDOWN)
	//~ boost::thread LegStatus_thread(callbackThread);
	ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
	
	int LegAngs[3] = {0};												// temp for holding leg angles
	ros::Rate loop_rate(LoopHz);
	//// Start up and stand ////
	while(ros::ok() && !SHUTDOWN)
	{
		switch(STATE)
		{
		  case 0: // start 
		  {
			//~ InverseKinematics(LegUpup_Car_Leg, LegAngs);			// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegUpUp_Ang_Leg, PosArray);		// move all leg groups
			PosArray.speed = 100;										// do first move slowly
			//~ PosArray = Angles2Joints(2, LegAngs, PosArray);			// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			usleep(1*1000*1000);
			SpiderRobotMain_pub.publish(PosArray);						// publish first command twice
			CurrentlyMoving = true;										// change status to moving as command was given
			STATE = 1;
			PosArray.speed = 300;
			break;
	  	  }
		  case 1:
		  {
			InverseKinematics(LegUpOut_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);				// move all leg groups
			WaitForDone();												// wait to be in position before moving again
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 2;
			break;
		  }
		  case 2:
		  {
			InverseKinematics(LegDownOut_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegAngs, PosArray);				// move all leg groups
			WaitForDone();												// wait to be in position before moving again
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 3;
			break;
		  }
		  case 3:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);				//  move leg group 1
			WaitForDone();												// wait to be in position before moving again
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 4;
			break;
		  }
		  case 4:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(1, LegAngs, PosArray);				//  move leg group 1
			WaitForDone();												// wait to be in position before moving agains
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 5;
			break;
		  }
		  case 5:
		  {
			InverseKinematics(LegUpIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);				//  move leg group 0
			WaitForDone();												// wait to be in position before moving again
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			STATE = 6;
			break;
		  }
		  case 6:
		  {
			InverseKinematics(LegDownIn_Car_Leg, LegAngs);				// find joint angles from general leg position
			PosArray = Angles2Joints(0, LegAngs, PosArray);				//  move leg group 0
			WaitForDone();												// wait to be in position before moving again
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			CurrentlyMoving = true;
			SHUTDOWN = true;											// stop while loop, startup is done, go to spin() now
			break;
		  }
		  default:
		  {
			PosArray.command = 1;										// command 1 is exit for serial controller
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			SHUTDOWN = true;											// use shutdown temp stop while loop
			break; break;												// exit
		  }
		}// end switch(STATE)
		loop_rate.sleep();
	}// while(ros::ok() && !SHUTDOWN) for startup and stand
	SHUTDOWN = false;													// true was just to exit last switch, make false again
	ROS_INFO("SpiderRobot standup complete");
	
	// make subscribing object for movement commands
	//~ ros::Subscriber RobotTwist_sub = nh.subscribe("RobotTwist", 1, RobotTwistCallback);
	//~ ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>("chatter", 1000,
																				//~ chatterCallbackCustomQueue,
																				//~ ros::VoidPtr(), &g_queue);
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>("RobotTwist", 1,
																				RobotTwistCallback,
																				ros::VoidPtr(), &g_queue);
	ros::Subscriber RobotTwist_sub = nh.subscribe(ops);					// put RobotTwist in custom queue (new thread)
	
	//~ ros::spin();													// dont use spin(), LegStatus will use spinOnce()
	RobotTwist_thread.join();											// start second thread?
	
	return 0;
}

/***********************************************************************************************************************
//The custom queue used for one of the subscription callbacks
***********************************************************************************************************************/
void callbackThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  ros::NodeHandle n;
  ros::Rate loop_rate(LoopHz);
  while (n.ok())
  {
    g_queue.callAvailable(ros::WallDuration(0.01));
    loop_rate.sleep();
  }
}

void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	//~ printf("Twist Status feedback: %f", twist->linear.x);
	ROS_INFO("Twist Status feedback: %f", twist->linear.x);
	usleep(1000*1000);
	// check for minimal value
	bool xyMove = false;												// checks for xy move to cancel theta move
	if( -1000 < twist->linear.x && twist->linear.x < 1000 )
	{
		Xstride = twist->linear.x/TwistFactor;
		printf("Xstride: %f\n", Xstride);
		xyMove = true;
	}
	if(-1000 < twist->linear.y && twist->linear.y < 1000)
	{
		Ystride = twist->linear.y/TwistFactor;
		xyMove = true;
	}
	if(twist->linear.x > 1000)
	{
		Tstride = twist->angular.x/TwistFactor;
	}
	
	printf("LegGroupTurn: %d\n", LegGroupTurn);
	if(xyMove)															// if the twist is good (out of dead zone)
	{
		short int res;													// holds results of function calls, pass/fail
		if(LegGroupTurn) //move group 1
		{
			//~ // move group 1 up and forward
			FPG1L0_C[0] = G1L0_Home_Car_Rob[0] + Xstride;				// move group 1 up and forward
			FPG1L0_C[1] = G1L0_Home_Car_Rob[1] + Ystride;				// group 0, leg 0
			FPG1L0_C[2] = G1L0_Home_Car_Rob[2] + Zstride;				// move up (default 1 cm)
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			TransferFrame(0, 3, FPG1L0_C);								// move points from robot frame to leg frame 0
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			res = InverseKinematics(FPG1L0_C, FPG1L0_A);				// find IK, need to check res more inthe future
			if(~res)
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			WaitForDone();												// wait for legs to reach position
			CurrentlyMoving = true;
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			LegGroupTurn = 0;
			
			// move group 0 back
			FPG0L0_C[0] = G0L0_Home_Car_Rob[0] - Xstride;				// move group 0 back
			FPG0L0_C[1] = G0L0_Home_Car_Rob[1] - Ystride;				// group 1, leg 0
			FPG0L0_C[2] = G0L0_Home_Car_Rob[2];
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			TransferFrame( 0, 0, FPG0L0_C);								// move points from robot frame to leg frame 0
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			res = InverseKinematics(FPG0L0_C, FPG0L0_A);				// find IK, need to check res more inthe future
			if(~res)	
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			LegGroupTurn = 1;
			WaitForDone();												// wait for legs up and back
			CurrentlyMoving = true;
			
			// move group 1 down
			FPG1L0_C[0] = G1L0_Home_Car_Rob[0] + Xstride;				// move group 1 down
			FPG1L0_C[1] = G1L0_Home_Car_Rob[1] + Ystride;				// group 0, leg 0
			FPG1L0_C[2] = G1L0_Home_Car_Rob[2];							// move back down
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			TransferFrame( 0, 3, FPG1L0_C);								// move points from robot frame to leg frame 0
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			res = InverseKinematics(FPG1L0_C, FPG1L0_A);				// find IK, need to check res more inthe future
			if(~res)
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			LegGroupTurn = 0;
		}// end if(LegGroupTurn) //move group 1
		else //move group 0
		{
			// move group 1 up and forward
			FPG0L0_C[0] = G0L0_Home_Car_Rob[0] + Xstride;				// move group 0 up and forward
			FPG0L0_C[1] = G0L0_Home_Car_Rob[1] + Ystride;				// group 0, leg 0
			FPG0L0_C[2] = G0L0_Home_Car_Rob[2] + Zstride;				// move up (default 1 cm)
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			TransferFrame( 0, 0, FPG0L0_C);								// move points from robot frame to leg frame 0
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			res = InverseKinematics(FPG0L0_C, FPG0L0_A);				// find IK, need to check res more inthe future
			if(~res)	
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			WaitForDone();												// wait for legs to reach position
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			LegGroupTurn = 1;
			
			
			FPG1L0_C[0] = G1L0_Home_Car_Rob[0] - Xstride;				// move group 1 bac
			FPG1L0_C[1] = G1L0_Home_Car_Rob[1] - Ystride;				// group 1, leg 0
			FPG1L0_C[2] = G0L0_Home_Car_Rob[2];
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			TransferFrame( 0, 3, FPG1L0_C);								// move points from robot frame to leg frame 0
			printf("FPG1L0_C: %f, %f, %f\n", FPG1L0_C[0], FPG1L0_C[1], FPG1L0_C[2]);
			res = InverseKinematics(FPG1L0_C, FPG1L0_A);				// find IK, need to check res more inthe future
			if(~res)
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			WaitForDone();												// wait for legs up and back
			CurrentlyMoving = true;
			LegGroupTurn = 0;
			
			// move group 0 down
			FPG0L0_C[0] = G0L0_Home_Car_Rob[0] + Xstride;				// move group 1 down
			FPG0L0_C[1] = G0L0_Home_Car_Rob[1] + Ystride;				// group 0, leg 0
			FPG0L0_C[2] = G0L0_Home_Car_Rob[2];							// move back down
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			TransferFrame( 0, 3, FPG1L0_C);								// move points from robot frame to leg frame 0
			printf("FPG0L0_C: %f, %f, %f\n", FPG0L0_C[0], FPG0L0_C[1], FPG0L0_C[2]);
			res = InverseKinematics(FPG0L0_C, FPG0L0_A);				// find IK, need to check res more inthe future
			if(~res)
			{
				PosArray.data[3] = FPG1L0_A[0];
				PosArray.data[4] = FPG1L0_A[1];
				PosArray.data[5] = FPG1L0_A[2];
			}
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			LegGroupTurn = 1;
		}// end else //move group 0
			
	}// end if(xyMove)	
	else //turn move
	{
		//turn move
	}// end else (xyMove) //turn move
	WaitForDone();														// wait for legs to reach position
	CurrentlyMoving = true;
}// end RobotTwistCallback

/***********************************************************************************************************************
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
* callback that recieves leg status and set globle bool variable 
* CurrentlyMoving so that other functions know
***********************************************************************************************************************/
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	//~ printf("Leg Status feedback: %c\n", msg->data);
	//~ printf("checking...\n");
	switch(msg->data)
	{
	  case '.': // Not moving 46
	  {
		CurrentlyMoving = false;
		break;
	  }
	  case '+': // Moving 43
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
		//~ printf("waiting... ");
		ros::spinOnce();												// Check for leg status msg
		//~ RobotTwist_thread.join();
		usleep(100*1000);												// wait 10th of a second
	}
	//~ printf("\ndone\n ");
	return 0;
}// end WaitForDone()

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DETECTED...\n");
	SHUTDOWN = true;
}

short int StrideGroupMove(short int Group, float X, float Y, float Z)
{
	
}
