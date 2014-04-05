#include "ros/ros.h"
#include <SpiderRobot_pkg/MyArray.h>

/***********************************************************************************************************************
void Angles2Joints(short int group, int Joints[3])
Takes array of desired leg joints and places them in PosArray.data for publishing
* int group will place the joints in the correct positions of PosArray.data for the desired leg group
* * group 0: places angles for leg group 0
* * group 1: places angles for leg group 1
* * group 2: places angles for leg group 0 and 1
***********************************************************************************************************************/
SpiderRobot_pkg::MyArray Angles2Joints(short int group, int Joints[], SpiderRobot_pkg::MyArray PosArray)
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
	 }// end case 2
	}// end switch
	
	return PosArray;
}// end Angles2Joints()
