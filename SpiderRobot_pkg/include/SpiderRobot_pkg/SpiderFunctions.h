#include <math.h>
#include "ros/ros.h"
#include "SpiderRobot_pkg/SpiderConstants.h"
#include <SpiderRobot_pkg/MyArray.h>


float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void MultiplyMat(float A[4][4], float B[4][1], float C[4][1]);//,int N, int L, int M);
short int InverseKinematics(float BasePoints[3], int LegAng[3]);
SpiderRobot_pkg::MyArray Angles2Joints(short int group, int Joints[], SpiderRobot_pkg::MyArray PosArray);

// homogeneous transfer frame matrics
float H_B_G0L0[4][4] =
{{.866025,      .5, 0, -.136906}
,{    -.5, .866025, 0,        0}
,{      0,       0, 1,    .0254}
,{      0,       0, 0,        1}};
float H_B_G0L1[4][4] =
{{      0,      -1, 0, -.136906}
,{      1,       0, 0,        0}
,{      0,       0, 1,    .0254}
,{      0,       0, 0,        1}};
float H_B_G0L2[4][4] =
{{-.866025,      .5, 0, -.136906}
,{     -.5,-.866025, 0,        0}
,{       0,       0, 1,    .0254}
,{       0,       0, 0,        1}};
float H_B_G1L0[4][4] =
{{.866025,     -.5, 0, -.136906}
,{     .5, .866025, 0,        0}
,{      0,       0, 1,    .0254}
,{      0,       0, 0,        1}};
float H_B_G1L1[4][4] =
{{      0,       1, 0, -.136906}
,{     -1,       0, 0,        0}
,{      0,       0, 1,    .0254}
,{      0,       0, 0,        1}};
float H_B_G1L2[4][4] =
{{-.866025,     -.5, 0, -.136906}
,{      .5,-.866025, 0,        0}
,{       0,       0, 1,    .0254}
,{       0,       0, 0,        1}};

/***********************************************************************************************************************
float* TransferFrame(short int Mode,short int Leg, float BasePoints[])
3 modes
Mode 0, single leg: Takes array of leg end effector position in base frame (robot), transfers it into leg frame (local) and returns
* input (short int Mode = 0, short int Leg, float BasePoints[Bx, By, Bz])
* * need leg number and position
* output float LocPoints[3]
Mode 1, Leg group 0: Takes array of 3 leg end effector positions in base frame, transfers them into leg frames (local) and returns
* input (short int Mode = 1, short int Leg = 0, float BasePoints[B0x, B0y, B0z, B1x, B1y, B1z, B2x, B2y, B2z])
* * need desired position in 1 array. Leg number does not matter
* output float LocPoints[9]
Mode 2, Leg group 1: Takes array of 3 leg end effector positions in base frame, transfers them into leg frames (local) and returns
* input (short int Mode = 2, short int Leg = 0, float BasePoints[B0x, B0y, B0z, B1x, B1y, B1z, B2x, B2y, B2z])
* * need desired position in 1 array. Leg number does not matter
* output float LocPoints[9]
***********************************************************************************************************************/
// returns LocPoints[]
float* TransferFrame(short int Mode,short int Leg, float BasePoints[])
{
	switch (Mode)
	{
	  case 0: // single leg transfer
	  {
		// float LocPoints[Lx, Ly, Lz] TransferFrame(short int Mode = 0, short int Leg, float BasePoints[Bx, By, Bz])
		float HPointB[4][1];							// homogeneous representation global point, colume verctor
		//float PointL[3x];								// local point
		HPointB[0][0] = BasePoints[0];
		HPointB[1][0] = BasePoints[1];
		HPointB[2][0] = BasePoints[2];
		HPointB[3][0] = 1;
		float HPointL[4][1] = {0};						// homogeneous representation local point to return
		
		switch (Leg)
		{
		  case 0:
		  {
			MultiplyMat(H_B_G0L0, HPointB, HPointL);
			break;
		  }
		  case 1:
		  {
			MultiplyMat(H_B_G0L1, HPointB, HPointL);		
			break;
		  }
		  case 2:
		  {
			MultiplyMat(H_B_G0L2, HPointB, HPointL);		
			break;
		  }
		  case 3:
		  {
			MultiplyMat(H_B_G1L0, HPointB, HPointL);
			break;
		  }
		  case 4:
		  {
			MultiplyMat(H_B_G1L1, HPointB, HPointL);	
			break;
		  }
		  case 5:
		  {
			MultiplyMat(H_B_G1L2, HPointB, HPointL);	
			break;
		  }
		}
		BasePoints[0] = HPointL[0][0];					// return BasePoint as LocPoint
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		return BasePoints;
		break;
	  }
	  case 1: // leg Group 0 transfer
	  {
		// float LocPoints[L0x, L0y, L0z, L1x, L1y, L1z, L2x, L2y, L2z] TransferFrame(short int Mode = 1, short int Leg = 0, float BasePoints[B0x, B0y, B0z, B1x, B1y, B1z, B2x, B2y, B2z])
		float HPointB[4][1];							// homogeneous representation global point
		float HPointL[4][1] = {0};						// homogeneous representation local point to return
		
		// first leg
		HPointB[0][0] = BasePoints[0];					// place in homo array
		HPointB[1][0] = BasePoints[1];
		HPointB[2][0] = BasePoints[2];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G0L0, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		// second leg
		HPointB[0][0] = BasePoints[3];					// place in homo array
		HPointB[1][0] = BasePoints[4];
		HPointB[2][0] = BasePoints[5];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G0L1, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		// third leg
		HPointB[0][0] = BasePoints[6];					// place in homo array
		HPointB[1][0] = BasePoints[7];
		HPointB[2][0] = BasePoints[8];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G0L2, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		return BasePoints;
		break;
	  }
	  case 2: // leg Group 1 transfer
	  {
		// float LocPoints[L0x, L0y, L0z, L1x, L1y, L1z, L2x, L2y, L2z] TransferFrame(short int Mode = 2, short int Leg = 0, float BasePoints[B0x, B0y, B0z, B1x, B1y, B1z, B2x, B2y, B2z])
		float HPointB[4][1];							// homogeneous representation global point
		float HPointL[4][1] = {0};						// homogeneous representation local point to return
		
		// first leg
		HPointB[0][0] = BasePoints[0];					// place in homo array
		HPointB[1][0] = BasePoints[1];
		HPointB[2][0] = BasePoints[2];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G1L0, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		// second leg
		HPointB[0][0] = BasePoints[3];					// place in homo array
		HPointB[1][0] = BasePoints[4];
		HPointB[2][0] = BasePoints[5];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G1L1, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		// third leg
		HPointB[0][0] = BasePoints[6];					// place in homo array
		HPointB[1][0] = BasePoints[7];
		HPointB[2][0] = BasePoints[8];
		HPointB[3][0] = 1;
		MultiplyMat(H_B_G1L2, HPointB, HPointL);		// calculate local
		BasePoints[0] = HPointL[0][0];					// place backint BasePoints as it becomes LocPoints before return
		BasePoints[1] = HPointL[1][0];
		BasePoints[2] = HPointL[2][0];
		
		return BasePoints;
		
		break;
	  }
	}
}


// http://www.cppforschool.com/tutorial/array2.html
void MultiplyMat(float A[4][4], float B[4][1], float res[4][1])//,int N, int L, int M)
{
  for(short int R=0;R<4;R++)
   for(short int C=0;C<1;C++)
   {
      res[R][C]=0;
      for(short int T=0;T<4;T++)
        res[R][C]+=A[R][T]*B[T][C];
    }
}


/***********************************************************************************************************************
float* InverseKinematics(float BasePoints[3])
Takes array of leg end effector position in leg frame, transfers it into jointspace and returns
* output float LegAng[3]
* returns int = 0 if successful
***********************************************************************************************************************/
short int InverseKinematics(float BasePoints[3], int LegAng[3])
{
	using namespace SpiderRobotConstants;
	
	// inverse k should be leg independent and not need modes
	// based on law of cosines
	// need to find theta0, theta1, and theta2
	float theta0, theta1, theta2;
	float D1;											// for inverse k
	float Ax;											// x and z points in xz plane
	float temp;											// holder
	
	// stage 1, find theta 0
	theta0 = atan2(BasePoints[1], BasePoints[0]);
	
	//~ printf("Ax y Az: %f, %f, %f\n", BasePoints[0], BasePoints[1], BasePoints[2]);
	
	// stage 2, find Ax in xz plane
	Ax = sqrt( pow( (float)BasePoints[0], 2.0) + pow( (float)BasePoints[1], 2.0) );
	//~ printf("Ax: %f\n", Ax);

	Ax = Ax - L0;										// add offset for J1 to J2 distance
	//~ printf("Ax: %f\n", Ax);
	
	// stage 3 find theta1 and theta2 from Az and BasePoints[2] or "Az"
	//~ D1 = ( Goal.x(2)^2 + Goal.y(2)^2 - L(1).leng^2 - L(2).leng^2 ) / (2*L(1).leng*L(2).leng); // from matlab
	D1 = pow(Ax, 2.0) + pow(BasePoints[2], 2.0) - pow(L1, 2.0) - pow(L2, 2.0);
	//~ printf("D1: %f... %f + %f - %f - %f\n", D1, pow(Ax, 2), pow(BasePoints[2], 2), pow(0.05715, 2), pow(0.127, 2) );
	D1 = D1 / (2.0*L1*L2);
	//~ printf("(2.0*0.05715*0.127): %f\n", (2.0*0.05715*0.127));
	//~ printf("D1: %f\n", D1);
	
	// L(2).angdes =  atan2( real(-sqrt(1-D1^2)),D1 ); // from matlab
	temp = 1 - pow(D1,2.0);
	if(temp > 0) 
	{
		theta2 = atan2( -sqrt(temp), D1);
	}
	else
	{
		ROS_ERROR("Bad value (1 - pow(D1,2) > 0) = false:: %f", temp);
		ROS_INFO("Theta0: %f, Ax: %f, D1: %f, temp: %f", theta0, Ax, D1, temp);
		ROS_INFO("BasePoints: %f, %f, %f", BasePoints[0], BasePoints[1], BasePoints[2]);
		return 1;
	}
	// angle offsets from home between matlab and physial robot ( 90 deg)
	//~ printf("theta2: %f\n", theta2);
	
	// L(1).angdes =  -(  atan2(Goal.y(2),Goal.x(2)) - atan2(L(2).leng*sin(L(2).angdes), L(1).leng + L(2).leng*cos(L(2).angdes))  )
	theta1 =  atan2(BasePoints[2], Ax) -  atan2(L2*sin(theta2), L1 + L2*cos(theta2));
	//~ printef("atan2(BasePoints[2], Ax): %f\n", atan2(BasePoints[2], Ax));
	//~ printf("atan2(0.127*sin(theta2), 0.05715 + 0.127*cos(theta2), Ax): %f\n", atan2(0.127*sin(theta2), 0.05715 + 0.127*cos(theta2)));
	//~ printf("0.127*sin(theta2): %f\n", 0.127*sin(theta2));
	//~ printf("0.05715 + 0.127*cos(theta2): %f\n", 0.05715 + 0.127*cos(theta2));
	
	
	
	//~ printf("theta1: %f\n", theta1);
	//~ printf("theta2: %f\n", theta2);
	theta2 = -1*(theta2 + PI/2.0);
	//~ printf("theta2: %f\n", theta2);

	//~ printf("Leg angles (deg): %d, %d, %d\n", theta0*180.0/PI, theta1*180.0/PI, theta2*180.0/PI);
	//~ printf("Leg angles (rad): %f, %f, %f\n", theta0, theta1, theta2);
	
	LegAng[0] = (int)(theta0*180.0/PI);
	LegAng[1] = (int)(theta1*180.0/PI);
	LegAng[2] = (int)(theta2*180.0/PI);
	printf("IK. Leg angles (deg): %d, %d, %d\n", LegAng[0], LegAng[1], LegAng[2]);
	
	return 0;
}

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
