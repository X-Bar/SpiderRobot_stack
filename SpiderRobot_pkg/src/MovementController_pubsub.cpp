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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>														// for abs
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
short int MoveLegGroupxy(short int LegGroup, float Xstride, float Ystride, float Zstride, int Speed);
short int MoveLegGroupT(short int LegGroup, float Tstride, float Zstride, int Speed);
short int PlanStrideMoveXY(short int Leg, float X, float Y, float Z);

using namespace SpiderRobotConstants;									// stores leg common leg positions

ros::CallbackQueue g_queue;												// custom queue for leg status feedback
boost::thread RobotTwist_thread(callbackThread);

bool CurrentlyMoving = true;											// flag of whether serial controller returns that its currenly moving
int STATE = 0;															// for state machine
bool SHUTDOWN = false;													// flag to shutdown while loop
SpiderRobot_pkg::MyArray PosArray;										// ROS message to publish
short int LegGroupTurn = 0;											// leg group 0 or 1's turn to move
float Zposition = 0;													// will eventually modify standing height
float Xstride = 0;														// walking stride in X, Y, Z, and rotation theta
float Ystride = 0;														
float Zstride = .015;													// amount legs left up while walking, default 1.5 cm
float Tstride = 0;														
float TwistFactor = 100;												// convert between raw twist and meter
short int LoopHz = 5;													// rate for loops
int PublishDelay = 300*1000;											// delay after publishing
int HighSpeed = 0;														// Speed for fast movements
int LowSpeed = 0;														// Speed for slow movements

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
	//~ ROS_INFO_STREAM("Main thread id=" << boost::this_thread::get_id());
	
	int LegAngs[3] = {0};												// temp for holding leg angles
	ros::Rate loop_rate(LoopHz);
	//// Start up and stand ////
	while(ros::ok() && !SHUTDOWN)
	{
		switch(STATE)
		{
		  case 0: // start 
		  {
			printf("Starting state...\n");
			// InverseKinematics(LegUpup_Car_Leg, LegAngs);			// find joint angles from general leg position
			PosArray = Angles2Joints(2, LegUpUp_Ang_Leg, PosArray);		// move all leg groups
			PosArray.speed = 100;										// do first move slowly
			//PosArray = Angles2Joints(2, LegAngs, PosArray);			// move all leg groups
			SpiderRobotMain_pub.publish(PosArray);						// publish command
			usleep(PublishDelay*10);
			SpiderRobotMain_pub.publish(PosArray);						// publish first command twice
			usleep(PublishDelay*10);
			CurrentlyMoving = true;										// change status to moving as command was given
			STATE = 1;
			PosArray.speed = 500;
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
		//~ 
		usleep(PublishDelay);
		loop_rate.sleep();
		
	}// while(ros::ok() && !SHUTDOWN) for startup and stand
	SHUTDOWN = false;													// true was just to exit last switch, make false again
	ROS_INFO("SpiderRobot standup complete");
	
	// make subscribing object for movement commands
	// ros::Subscriber RobotTwist_sub = nh.subscribe("RobotTwist", 1, RobotTwistCallback);
	// ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::String>("chatter", 1000,
																				//~ chatterCallbackCustomQueue,
																				//~ ros::VoidPtr(), &g_queue);
	ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>("RobotTwist", 1,
																				RobotTwistCallback,
																				ros::VoidPtr(), &g_queue);
	ros::Subscriber RobotTwist_sub = nh.subscribe(ops);					// put RobotTwist in custom queue (new thread)
	
	// ros::spin();													// dont use spin(), LegStatus will use spinOnce()
	RobotTwist_thread.join();											// start second thread?
	
	return 0;
}

/***********************************************************************************************************************
//The custom queue used for one of the subscription callbacks
***********************************************************************************************************************/
void callbackThread()
{
	//~ ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
	
	usleep(2*1000*1000);												// wait for rosinit
	ros::NodeHandle n;
	ros::Rate loop_rate(LoopHz);
	while (n.ok())
	{
		g_queue.callAvailable(ros::WallDuration(0.01));					// like soin()
		loop_rate.sleep();
	}
}

void RobotTwistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
	 // printf("Twist Status feedback: %f", twist->linear.x);
	ROS_INFO("Twist Status feedback: %f", twist->linear.x);
	// check for minimal value
	HighSpeed = 0;
	LowSpeed = 0;
	bool xyMove = false;												// checks for xy move, cancelss theta move
	bool tMove = false;												// checks for theta move
	short int res;														// holds result
	
	// logic, find what type of move we need
	if( twist->linear.x <= -.1 ||  .1 <= twist->linear.x )				// check minmal value
	{
		if( -3.5 <= twist->linear.x && twist->linear.x  <= 3.5 )		// check maximium value
		{
			Xstride = twist->linear.x/TwistFactor;
			printf("Xstride: %f\n", Xstride);
			HighSpeed = abs( (int)(100.0*twist->linear.x) );
			LowSpeed = abs( (int)(50.0*twist->linear.x) );
			xyMove = true;
		}	
	}
	if( twist->linear.y <=  -.1 ||  .1 <= twist->linear.y )				// check minmal value
	{
		if( -3.5 <= twist->linear.y && twist->linear.y  <= 3.5 )		// check maximium value
		{
			Ystride = twist->linear.y/TwistFactor;
			HighSpeed += abs( (int)(100*twist->linear.y) );
			LowSpeed += abs( (int)(50*twist->linear.y) );
			xyMove = true;
		}
	}
	if(~xyMove)															// if no xy move
	{
		if( twist->linear.y <=  -.1 ||  .1 <= twist->linear.y )			// check minmal value
		{
			if( -3.5 <= twist->linear.y && twist->linear.y  <= 3.5 )
			{
				Tstride = twist->angular.x/TwistFactor;
				tMove = true;											// there is a theta move now
			}
		}
		
	}
	
	// move
	if(xyMove)															// if the twist is good (out of dead zone)
	{
		//~ // printf("LegGroupTurn: %d\n", LegGroupTurn);
																		// holds results of function calls, pass/fail
		if(LegGroupTurn) //move group 1
		{
			//~ // move group 1 up and forward
			//~ short int MoveLegGroupxy(short int LegGroup, float Xstride, float Ystride, float Zstride, int Speed)
			res = MoveLegGroupxy(1, Xstride, Ystride, Zstride, HighSpeed);
			usleep(PublishDelay);
			
			//~ // move group 0 home
			res = MoveLegGroupxy(0, 0, 0, 0, LowSpeed);
			usleep(2*PublishDelay);
			
			//~ // move group 0 back
			res = MoveLegGroupxy(0, (-1)*Xstride, (-1)*Ystride, 0, LowSpeed);
			usleep(PublishDelay);
			
			//~ // move group 1 down
			res = MoveLegGroupxy(1, Xstride, Ystride, 0, HighSpeed);
			usleep(PublishDelay);
			
			LegGroupTurn = 0;
		}// end if(LegGroupTurn) //move group 1
		else //move group 0
		{
			//~ // move group 0 up and forward
			res = MoveLegGroupxy(0, Xstride, Ystride, Zstride, HighSpeed);
			usleep(PublishDelay);
			
			//~ // move group 1 home
			res = MoveLegGroupxy(1, 0, 0, 0, LowSpeed);
			usleep(PublishDelay);
			
			//~ // move group 1 back
			res = MoveLegGroupxy(1, (-1)*Xstride, (-1)*Ystride, 0, LowSpeed);
			usleep(PublishDelay);
			
			//~ // move group 0 down
			res = MoveLegGroupxy(0, Xstride, Ystride, 0, HighSpeed);
			usleep(PublishDelay);
			
			LegGroupTurn = 1;
		}// end else //move group 0
		usleep(PublishDelay);
	}// end if(xyMove)	
	else if(tMove) //turn move
	{
		//~ // move group 1 up and forward
		res = MoveLegGroupT(1, Tstride, Zstride, HighSpeed);
		usleep(PublishDelay);
		
		//~ // move group 0 home
		res = MoveLegGroupT(0, 0, 0, LowSpeed);
		usleep(2*PublishDelay);
		
		//~ // move group 0 back
		res = MoveLegGroupT(0, (-1)*Tstride, 0, LowSpeed);
		usleep(PublishDelay);
		
		//~ // move group 1 down
		res = MoveLegGroupT(1, Tstride, 0, HighSpeed);
		usleep(PublishDelay);
		
		LegGroupTurn = 0;
		
	}// end else if(tMove) //turn move
}// end RobotTwistCallback

/***********************************************************************************************************************
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
* callback that recieves leg status and set globle bool variable 
* CurrentlyMoving so that other functions know
***********************************************************************************************************************/
void LegStatusCallback(const std_msgs::Char::ConstPtr& msg)
{
	printf("Leg Status feedback: %c\n", msg->data);
	//~ printf("checking...\n");
	switch(msg->data)
	{
	  case '.': // Not moving 46
	  {
		CurrentlyMoving = false;
		printf("Not moving\n");
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
		printf("waiting... ");
		ros::spinOnce();												// Check for leg status msg
		usleep(100*1000);												// wait 10th of a second
	}
	printf("\ndone\n ");
	return 0;
}// end WaitForDone()

void shutdownHandler(int s)
{
	ROS_INFO("SHUTDOWN COMMAND DETECTED...\n");
	printf("SHUTDOWN COMMAND DETECTED...\n");
	SHUTDOWN = true;
}

short int MoveLegGroupxy(short int LegGroup, float Xstride, float Ystride, float Zstride, int Speed)
{
	int res;
	switch(LegGroup)
	{
		case 0:
		{
			res = PlanStrideMoveXY(0, Xstride, Ystride, Zstride);		// 0 = G0L0
			res = PlanStrideMoveXY(1, Xstride, Ystride, Zstride);		// 1 = G0L1
			res = PlanStrideMoveXY(2, Xstride, Ystride, Zstride);		// 2 = G0L2
			PosArray.speed = Speed;
			WaitForDone();												// wait for legs to reach position
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			CurrentlyMoving = true;
			break;
		}
		case 1:
		{
			res = PlanStrideMoveXY(3, Xstride, Ystride, Zstride);		// 3 = G1L0
			res = PlanStrideMoveXY(4, Xstride, Ystride, Zstride);		// 4 = G1L1
			res = PlanStrideMoveXY(5, Xstride, Ystride, Zstride);		// 5 = G1L2
			PosArray.speed = Speed;
			WaitForDone();												// wait for legs to reach position
			SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
			CurrentlyMoving = true;
			break;
			
		}
	} // end switch(LegGroup)
	return res;
}

short int MoveLegGroupT(short int LegGroup, float Tstride, float Zstride, int Speed)
{
	int res;
	// calculate R in robot frame based on G1L1 or leg 4
	float Y = RobotR + FPG1L1_C[1];										// base R +  last leg(y)
	float X = FPG1L1_C[0];												// leg(x)
	float R = pow( pow(Y, 2.0)+pow(X, 2.0) , .5);
	
	// calculate FP (foot points) in Robot frame for a single leg G1L1
	Y = -1*R*sin(Tstride);
	X = R*cos(Tstride);
	
	float FP_C[3];
	int FP_A[3];														// foot  point
	FP_C[0] = X;														// move group to stride position
	FP_C[1] = Y;
	FP_C[2] = G1L1_Home_Car_Rob[2] + Zstride;							// move up or back down (default 2 cm)
	TransferFrame(0, 4, FP_C);											// move points from robot frame to leg frame 4 (G1L1)
	res = InverseKinematics(FP_C, FP_A);								// find IK, need to check res more inthe future
	
	if(res)
		return 1;
	
	switch(LegGroup)
	{
		case 0:
		{
			PosArray.data[0] = FP_A[0];									// leg 0 G0L0
			PosArray.data[1] = FP_A[1];
			PosArray.data[2] = FP_A[2];
			
			PosArray.data[12] = FP_A[0];								// leg 1 G0L1
			PosArray.data[13] = FP_A[1];
			PosArray.data[14] = FP_A[2];
			
			PosArray.data[6] = FP_A[0];									// leg 2 G0L2
			PosArray.data[7] = FP_A[1];
			PosArray.data[8] = FP_A[2];
			break;
		}
		case 1:
		{
			PosArray.data[9] = FP_A[0];									// leg 3 G1L0
			PosArray.data[10] = FP_A[1];
			PosArray.data[11] = FP_A[2];
			
			PosArray.data[3] = FP_A[0];									// leg 4 G1L1
			PosArray.data[4] = FP_A[1];
			PosArray.data[5] = FP_A[2];
			
			PosArray.data[15] = FP_A[0];								// leg 5 G1L2
			PosArray.data[16] = FP_A[1];
			PosArray.data[17] = FP_A[2];
			break; 
		}
	} // end switch(LegGroup)
	
	PosArray.speed = Speed;
	WaitForDone();												// wait for legs to reach position
	SpiderRobotMain_pub.publish(PosArray);						// publish command to lift legs up
	CurrentlyMoving = true;
	
	return res;
}

short int PlanStrideMoveXY(short int Leg, float X, float Y, float Z)
{
	int res = 1;														// result
	switch (Leg)
	{
	  case 0:
	  {
		FPG0L0_C[0] = G0L0_Home_Car_Rob[0] + X;							// move group to stride position
		FPG0L0_C[1] = G0L0_Home_Car_Rob[1] + Y;
		FPG0L0_C[2] = G0L0_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG0L0_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG0L0_C, FPG0L0_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[0] = FPG0L0_A[0];
			PosArray.data[1] = FPG0L0_A[1];
			PosArray.data[2] = FPG0L0_A[2];
		}
		break;
	  }
	  case 1:
	  {
		//~ MultiplyMat(H_B_G0L1, HPointB, HPointL);	
		FPG0L1_C[0] = G0L1_Home_Car_Rob[0] + X;							// move group to stride position
		FPG0L1_C[1] = G0L1_Home_Car_Rob[1] + Y;
		FPG0L1_C[2] = G0L1_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG0L1_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG0L1_C, FPG0L1_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[12] = FPG0L1_A[0];
			PosArray.data[13] = FPG0L1_A[1];
			PosArray.data[14] = FPG0L1_A[2];
		}	
		break;
	  }
	  case 2:
	  {
		//~ MultiplyMat(H_B_G0L2, HPointB, HPointL);		
		FPG0L2_C[0] = G0L2_Home_Car_Rob[0] + X;							// move group to stride position
		FPG0L2_C[1] = G0L2_Home_Car_Rob[1] + Y;
		FPG0L2_C[2] = G0L2_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG0L2_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG0L2_C, FPG0L2_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[6] = FPG0L2_A[0];
			PosArray.data[7] = FPG0L2_A[1];
			PosArray.data[8] = FPG0L2_A[2];
		}	
		break;
	  }
	  case 3:
	  {
		//~ MultiplyMat(H_B_G1L0, HPointB, HPointL);
		FPG1L0_C[0] = G1L0_Home_Car_Rob[0] + X;							// move group to stride position
		FPG1L0_C[1] = G1L0_Home_Car_Rob[1] + Y;
		FPG1L0_C[2] = G1L0_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG1L0_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG1L0_C, FPG1L0_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[9] = FPG1L0_A[0];
			PosArray.data[10] = FPG1L0_A[1];
			PosArray.data[11] = FPG1L0_A[2];
		}	
		break;
	  }
	  case 4:
	  {	
		FPG1L1_C[0] = G1L1_Home_Car_Rob[0] + X;							// move group to stride position
		FPG1L1_C[1] = G1L1_Home_Car_Rob[1] + Y;
		FPG1L1_C[2] = G1L1_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG1L1_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG1L1_C, FPG1L1_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[3] = FPG1L1_A[0];
			PosArray.data[4] = FPG1L1_A[1];
			PosArray.data[5] = FPG1L1_A[2];
		}	
		break;
	  }
	  case 5:
	  {
		FPG1L2_C[0] = G1L2_Home_Car_Rob[0] + X;							// move group to stride position
		FPG1L2_C[1] = G1L2_Home_Car_Rob[1] + Y;
		FPG1L2_C[2] = G1L2_Home_Car_Rob[2] + Z;							// move up or back down (default 2 cm)
		TransferFrame(0, Leg, FPG1L2_C);								// move points from robot frame to leg frame 0
		res = InverseKinematics(FPG1L2_C, FPG1L2_A);					// find IK, need to check res more inthe future
		if(~res)
		{
			PosArray.data[15] = FPG1L2_A[0];
			PosArray.data[16] = FPG1L2_A[1];
			PosArray.data[17] = FPG1L2_A[2];
		}	
		break;
	  }
	} // end switch (Leg)
	return res;
}
