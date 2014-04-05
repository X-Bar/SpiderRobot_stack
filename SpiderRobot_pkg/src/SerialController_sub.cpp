/***********************************************************************************************************************
FILENAME:   SerialController.cpp
AUTHORS:    Cody L. Lundberg based on code from Christopher D. McMurrough (base_controller.cpp for IGVC)

DESCRIPTION:
ROS interface node serial commands. Recieves topic, converts to serial commands and sends them. Currently in use with SSC-32 servo controller

PUBLISHES:  NA
SUBSCRIBES: 
SERVICES:   NA

REVISION HISTORY:
05.06.2013   CDM     Christopher D. McMurrough original file creation
05.17.2013           Cody L. Lundberg reuse. Focus on making code more general purpose.
***********************************************************************************************************************/

#include "ros/ros.h"
//#include "std_msgs/Int16.h"
//#include "std_msgs/String.h"
#include "std_msgs/Char.h"
//#include "std_msgs/Bool.h"
//#include <geometry_msgs/TwistStamped.h>
//#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
//#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <math.h>

#include <SpiderRobot_pkg/MyArray.h>
#include <SpiderRobot_pkg/My2Num.h>

#define SERIAL_PACKET_START_CHAR 0xAA
#define SERIAL_PACKET_LENGTH 0x08
#define PI 3.14159265

ros::Time LAST_COMMAND_TS;
bool SHUTDOWN = false;

// define the timeout duration
double TIMEOUT_SECONDS = 0.5;

// serial port handler
int serialPort;

void motionCommandCallback(const SpiderRobot_pkg::MyArray::ConstPtr& msg);
void SingleCommandCallback(const SpiderRobot_pkg::My2Num::ConstPtr& msg);
//void motionCommandCallback(const std_msgs::Int16MultiArray::ConstPtr& JointPositionsArray);
void CheckPos(short int Mode, int *Pos, short int Cha, int *Pos1);
int openSerialPort(char* portName);
void closeSerialPort(int serialPort);
void shutdownHandler(int s);

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, expects the serial port name and baud rate as command line arguments
***********************************************************************************************************************/
int main(int argc, char** argv)
{
	printf("Starting SerialController_sub \n");

	// store the serial port parameters
	char* portName;

	// validate and parse the command line arguments
	/*
	if(argc != 2)
	{
		ROS_WARN("USAGE: %s <port_name> \n", argv[0]);
		return 0;
	}
	*/

	char buffer[30] = {'\0'};							// buffer for send recieve data
	int n = 0, i;									// stores result from serial write/read, counter i
	std_msgs::Char MotorResponse;							// To publish for motor check
	portName = "/dev/ttyUSB0";
	//~ portName = "/dev/hiddev0";

	// attempt to open the serial port
	serialPort = openSerialPort(portName);

	// check to see if we connected successfully
	if(serialPort == -1)
	{
		printf("unable to open serial port %s \n", portName);
		return(0);
	}
	else
	{
		printf("serial port opened: %s \n", portName);
	}

	// set up the shutdown handler
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = shutdownHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);


	// initialize the ROS node
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle nh;

	// shutdown the device until a command is received
	LAST_COMMAND_TS = ros::Time::now() - ros::Duration(TIMEOUT_SECONDS);
	//stopMotors(serialPort);

	// Start publisher (feedback)
	ros::Publisher LegStatus_pub = nh.advertise<std_msgs::Char>("LegStatus", 1);

	// set the loop rate to 20 Hz	
	ros::Rate loop_rate(50);

	// start the subscriber
	SpiderRobot_pkg::MyArray PosArray;
	ros::Subscriber motionCommandSubscriber = nh.subscribe("MyArray", 1, motionCommandCallback);
	ros::Subscriber SingleCommand_sub = nh.subscribe("SingleCommand", 1, SingleCommandCallback);
	ros::spinOnce();


	// listen for message until program termination
	while(ros::ok() && !SHUTDOWN)
	{
		ros::spinOnce();
		// perform one iteration of message checking
		ros::spinOnce();

		// check for received serial data
		buffer[0] ='Q';								// Query command
		buffer[1] = 13;								// acsii carrier return for end of command
		int result = write(serialPort, buffer, 2);			// send to ssc-32
		//~ printf("bits sent: %d\n", result);					// print bits sent 
		memset(buffer,0,sizeof(buffer));					// clear buffer

		n = read(serialPort, buffer, sizeof(buffer));			// attempt to read bytes from the port
		if(n > 0)									// if reponse print any received bytes to terminal
		{
			//~ ROS_INFO("\nRECEIVED TTY RESPONSE...");
			for(i=0; i<n; i++)
			{
				//~ printf("%c", buffer[i]);
			}
			//~ printf(" \n\n");

			MotorResponse.data = buffer[0];
			LegStatus_pub.publish(MotorResponse);
		}
		else
		{
			//~ printf("No response to query");
		}


        loop_rate.sleep();								// sleep to maintain the loop rate
    }// end while

    closeSerialPort(serialPort);							// close the serial port
}// end main

//void motionCommandCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
void motionCommandCallback(const SpiderRobot_pkg::MyArray::ConstPtr& msg)
{
	switch(msg->command)
	{
	 case 0: // Update all joint commands
	 {											// command case 0: send command to servos, most common case
		char buffer[300], temp[10] = {'\0'};				// buffer for serial commands, and temp buffer
		int i, n, pos[18], uSecPos[18], result;				// counter i, size n, position commands in degree, position commands in usec, result from sending serial data
		//~ printf("\nRecived data for all channels...\n");
		for(i=0 ; i<18; i++)								// get position values
		{
			pos[i] = msg->data[i];
		}

		for(i=0 ; i<18; i++)
		{
			uSecPos[i] = (int)(11.3333* ((float)pos[i]) + 1500);	// convert to useconds of duty cycle
		}

		CheckPos(0, uSecPos, NULL, NULL);					// check to make sure desired position is in range of possible position, mode 0 for all channels

		sprintf(temp, "#0P%d", uSecPos[0]);					// convert to string for first motor (0)
		strcat(buffer, temp);
		for(i=1 ; i<18; i++)
		{
			if(i < 9)										// motor number, channel offset
			{
				sprintf(temp, "#%dP%d", i, uSecPos[i]);		// convert to string for the rest of the motors (i) with command uSecPos
			}
			else if(8 < i && i < 13)
			{
				sprintf(temp, "#%dP%d", i+7, uSecPos[i]);		// +7 is the channel motor offset
			}
			else //(12 < i && i < 29)
			{
				sprintf(temp, "#%dP%d", i+11, uSecPos[i]);	// +11 is the channel motor offset
			}
			strcat(buffer, temp);						// keep appending commands
			memset(temp, '\0', sizeof(temp));				// clear temp
		}	
	    if(msg->speed > 0 && 1000 > msg->speed)
	    {
	    	sprintf(temp, "S%d", msg->speed);					// add speed to array if it exists
	    }
	    else
	    {
		sprintf(temp, "S250");							// default if it doesnt exist
	    }
	    strcat(buffer, temp);

	    n = strlen(buffer);
	    //~ printf("size: %d\n", n);
	    //~ for(i = 0 ; i < n ; i++)							// print full command for checking
		//~ printf("%c", buffer[i]);
	    //~ printf("\n");

	    buffer[n] = 13;								// ascii carrier return, used for end bit. Should also remove null
	    result = write(serialPort, buffer, n+1);				// send to ssc-32
	    //~ printf("bits sent: %d\n", result);					// print bits sent

	    break;
	}

	 case 1:
	 {
	    printf("Emergency Stop Mode!!! \n Not yet functional, requires additional hardware to shutdown motors. Currently just shuts down the node\n");
	    SHUTDOWN = true;							
	    break; break;
	 }

	}
    LAST_COMMAND_TS = ros::Time::now();
}// end motionCommandCallback

void SingleCommandCallback(const SpiderRobot_pkg::My2Num::ConstPtr& msg)
{

	char buffer[20] = {'\0'}, temp[10] = {'\0'};				// buffer for serial commands, and temp buffer
	int i, n, uSecPos, result;
	//~ printf("\nRecived single channel data...\n");

	uSecPos = (int)(11.3333* ((float)msg->pos) + 1500);	// convert to useconds of duty cycle
	CheckPos(1, NULL, msg->cha, &uSecPos);					// check to make sure desired position is in range of possible position, mode 1 for only 1 channel

	sprintf(temp, "#%dP%d",msg->cha, uSecPos);				// convert to string for first motor (0)
	strcat(buffer, temp);

	strcat(buffer, "S250");								// set slow speed

	n = strlen(buffer);
	//~ printf("size: %d\n", n);
	//~ for(i = 0 ; i < n ; i++)							// print full command for checking
	//~ printf("%c", buffer[i]);
	//~ printf("\n");

	buffer[n] = 13;									// ascii carrier return, used for end bit. Should also remove null
	result = write(serialPort, buffer, n+1);					// send to ssc-32
	//~ printf("bits sent: %d\n", result);						// print bits sent



}// end SingleCommandCallback


/***********************************************************************************************************************
void CheckPos(int *Pos)
2 modes
Mode 0: Takes array of desired angle positions, checks them against known bad angles for safety.
Mode 1: Takes position  and channel and checks for bad angels
***********************************************************************************************************************/
void CheckPos(short int Mode, int *Pos, short int Cha, int *Pos1)
{
	switch(Mode)
	{
		case 0: // check all 18 joint commands
		{
			int i;
			for(i=0; i<18; i++)
			{
				if( i == 0 || i == 3 || i == 6 || i == 9 || i == 12 || i == 15 )	// first joint
				{
					if(Pos[i] < 1050)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 1050;
						printf("angle %d changed\n", i);
					}
					else if(Pos[i] > 1950)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 1950;
						printf("angle %d changed\n", i);
					}
				}
				if( i == 1 || i == 4 || i == 7 || i == 10 || i == 13 || i == 16 )	// second joint
				{
					if(Pos[i] < 1100)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 1250;
						printf("angle %d changed\n", i);
					}
					else if(Pos[i] > 2250)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 2250;
						printf("angle %d changed\n", i);
					}
				}
				if( i == 2 || i == 5 || i == 8 || i == 11 || i == 14 || i == 17 )	// third joint
				{
					if(Pos[i] < 600)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 600;
						printf("angle %d changed\n", i);
					}
					else if(Pos[i] > 1850)
					{
						printf("angle %d was %d\n", i, Pos[i]);
						Pos[i] = 1750;
						printf("angle %d changed\n", i);
					}
				}
			}// end forloop
			break;
		}//end case 0
		case 1: // check only 1 joint command
		{
			if( Cha == 0 || Cha == 3 || Cha == 6 || Cha == 9 || Cha == 12 || Cha == 15 )	// first joint
			{
				if(*Pos1 < 1050)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 1050;
					printf("angle %d changed\n", Cha);
				}
				else if(*Pos1 > 1950)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 1950;
					printf("angle %d changed\n", Cha);
				}
			}
			if( Cha == 1 || Cha == 4 || Cha == 7 || Cha == 10 || Cha == 13 || Cha == 16 )	// second joint
			{
				if(*Pos1 < 1250)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 1100;
					printf("angle %d changed\n", Cha);
				}
				else if(*Pos1 > 2250)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 2250;
					printf("angle %d changed\n", Cha);
				}
			}
			if( Cha == 2 || Cha == 5 || Cha == 8 || Cha == 11 || Cha == 14 || Cha == 17 )	// third joint
			{
				if(*Pos1 < 600)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 600;
					printf("angle %d changed\n", Cha);
				}
				else if(*Pos1 > 1850)
				{
					printf("angle %d was %d\n", Cha, *Pos1);
					*Pos1 = 1750;
					printf("angle %d changed\n", Cha);
				}
			}
			break;
		}// end case 1
	}// end switch
}// end CheckPos

/***********************************************************************************************************************
int openSerialPort(char* portName)
Opens a serial port given the name. Does some error checking, returns -1 if unsuccessful
***********************************************************************************************************************/
int openSerialPort(char* portName)
{
    // store the file descriptor for the serial port
    int fd;

    // attempt to open the port
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    // return -1 if we are unable to open the port
    if(fd == -1)
    {
        return -1;
    }
    else
    {
        // clear any existing file descriptor flags
        fcntl(fd, F_SETFL, 0);

        // create a structure to store the port settings
        struct termios port_settings;

        // set the baud rates
        cfsetispeed(&port_settings, B115200);
        cfsetospeed(&port_settings, B115200);

	/*
        // set no parity, stop bits, data bits
        port_settings.c_cflag &= ~PARENB;
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;
	*/
	port_settings.c_iflag = IGNBRK | IGNPAR;
        port_settings.c_oflag = 0;
        port_settings.c_cflag = B115200 | CREAD | CS8 | CLOCAL;
        port_settings.c_lflag = 0;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &port_settings);

        memset(&port_settings, 0, sizeof(port_settings));
        tcgetattr(fd, &port_settings);

        // set the port to use all 8 bits
        port_settings.c_iflag &= ~ISTRIP;

        // apply the settings to the port
        tcsetattr(fd, TCSANOW, &port_settings);

        // set the non blocking functionality
        fcntl(fd, F_SETFL, O_NONBLOCK);

        // return the file descriptor
        return(fd);
    }
}// end openSerialPort

/***********************************************************************************************************************
void closeSerialPort(int serialPort)
close the given serial port
***********************************************************************************************************************/
void closeSerialPort(int serialPort)
{
    tcflush(serialPort, TCIOFLUSH);
    close(serialPort);
}

/***********************************************************************************************************************
void shutdownHandler(int s)
send a stop motor command to the controller
***********************************************************************************************************************/
void shutdownHandler(int s)
{
    printf("exiting\n");
    ROS_INFO("SHUTDOWN COMMAND DETECTED...");
    SHUTDOWN = true;
}





