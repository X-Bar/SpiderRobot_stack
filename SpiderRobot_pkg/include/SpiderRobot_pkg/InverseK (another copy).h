#include <math.h>
#include "SpiderRobot_pkg/SpiderConstants.h"


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
