#ifndef MYLIB_CONSTANTS_H
#define MYLIB_CONSTANTS_H

//  File Name : LibConstants.hpp    Purpose : Global Constants for Lib Utils
namespace SpiderRobotConstants
{
	float L0 = .0381;													// link 0 length
	float L1 = .05715;													// link 1 length
	float L2 = .127;													// link 2 length
	float PI = 3.1416;
	
	// number of general positions, old version. Angle coordinates, leg frame 
	//  side/leg/joint  CurrentlyMoving   L/1/1      L/1/3   L/2/2      L/3/1     L/3/3     R/1/2     R/2/1     R/2/3     R/3/2                      
	//  side/leg/joint          L/1/2     L/2/1     L/2/3     L/3/2     R/1/1     R/1/3     R/2/2     R/3/1    R/3/3                     
	//  channel      Lside	 	0    1    2    3    4    5    6    7     8  RS9  10   11   12    13  14    15  16    17
	int AllLegsUpUp_Ang_Leg[] = {0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80,  0,  63,  -80};
	int AllLegsHome_Ang_Leg[] = {0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0,  0,   0,    0};
	int LegG0Down_Ang_Leg[] = 	{0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0};
	int LegG0Up_Ang_Leg[] = 	{0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25,  0,   0,    0};
	int LegG1Down_Ang_Leg[] = 	{0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15,  0,   0,    0,  0, -22,  -15};
	int LegG1Up_Ang_Leg[] = 	{0,   0,    0,  0,  27,   27,  0,   0,    0,  0,  27,   25,  0,   0,    0,  0,  27,   25};

	// number of general positions, new version. Use with function Angles to Joints
	// Angle coordinates, leg frame
	//						J0  J1  J2
	int LegUpUp_Ang_Leg[] =	{0, 60, -70};								// position for leg group all straight up
	int LegHome_Ang_Leg[] =	{0, 0, 0};									// leg all at zero
	int LegDown_Ang_Leg[] =	{0, -30, -35};								// legs in a standing down position
	int LegUp_Ang_Leg[] =	{0, 27, 25};								// legs in a lefted position

	// General positions. Cartesian coordinates, leg frame 
	//							X		Y	 	Z
	float LegHome_Car_Leg[] = {.09525, .00, -.127};
	float LegUpOut_Car_Leg[] =   {.15, .00, -.06};
	float LegUpIn_Car_Leg[] =   {.10, .00, -.10};
	float LegUpup_Car_Leg[] = {.10, .00, .06};
	float LegDownOut_Car_Leg[] = {.15, .00, -.12};
	float LegDownIn_Car_Leg[] = {.10, .00, -.15};
	
	// Home positions. Cartesian coordinates, robot frame 
	//							X		Y	 	Z
	float G0L0_Home_Car_Rob[3] = { .2011, .1161, -.1524};
	float G0L1_Home_Car_Rob[3] = { .0   ,-.23215, -.1524};
	float G0L2_Home_Car_Rob[3] = {-.2011, .1161, -.1524};
	float G1L0_Home_Car_Rob[3] = { .2011,-.1161, -.1524};
	float G1L1_Home_Car_Rob[3] = { .0   , .23215, -.1524};
	float G1L2_Home_Car_Rob[3] = {-.2011,-.1161, -.1524};
	
}
#endif