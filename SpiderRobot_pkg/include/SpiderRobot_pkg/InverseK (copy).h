#include <math.h>


/***********************************************************************************************************************
float* InverseKinematics(float BasePoints[3])
Takes array of leg end effector position in leg frame, transfers it into jointspace and returns
* output float LegAng[3]
* returns int = 0 if successful
***********************************************************************************************************************/
short int InverseKinematics(float BasePoints[3], int LegAng[3])
{
	// inverse k should be leg independent and not need modes
	// based on law of cosines
	// need to find theta0, theta1, and theta2
	float theta0, theta1, theta2;
	float D1;											// for inverse k
	float Ax;											// x and z points in xz plane
	float temp;											// holder
	float PI = 3.1416;
	// stage 1, find theta 0
	theta0 = atan2(BasePoints[1], BasePoints[0]);
	
	// stage 2, find Ax in xz plane
	Ax = sqrt( pow(BasePoints[0], 2) + pow(BasePoints[1], 2) );

	Ax = Ax - .03378;	// add offset for J1 to J2 distance
	
	// stage 3 find theta1 and theta2 from Az and BasePoints[2] or "Az"
	// D1 = ( Goal.x(2)^2 + Goal.y(2)^2 - L(1).leng^2 - L(2).leng^2 ) / (2*L(1).leng*L(2).leng); // from matlab
	D1 = pow(Ax, 2) + pow(BasePoints[2], 2) - pow(0.05715, 2) - pow(0.127, 2);
	D1 = D1 / (2.0*0.05715*0.127);
	
	// L(2).angdes =  atan2( real(-sqrt(1-D1^2)),D1 );
	temp = 1 - pow(D1,2);
	if(temp > 0) 
	{
		theta2 = atan2( -sqrt(temp), D1);
	}
	else
	{
		ROS_ERROR("Bad value (1 - pow(D1,2) > 0) = false:: %f\n", temp);
		ROS_INFO("Theta0: %f, Ax: %f, D1: %f, temp: %f\n", theta0, Ax, D1, temp);
		ROS_INFO("BasePoints: %f, %f, %f\n", BasePoints[0], BasePoints[1], BasePoints[2]);
		return 1;
	}
	
	// L(1).angdes =  atan2(Goal.y(2),Goal.x(2)) - atan2(L(2).leng*sin(L(2).angdes), L(1).leng + L(2).leng*cos(L(2).angdes))
	theta1 = atan2(BasePoints[2], Ax) - atan2(0.127*sin(theta2), 0.05715 + 0.127*cos(theta2));
	printf("Leg angles: %f, %f, %f\n", theta0*180.0/PI, theta1*180.0/PI, theta2*180.0/PI);
	
	LegAng[0] = (int)(theta0*180.0/PI);
	LegAng[1] = (int)(theta1*180.0/PI);
	LegAng[2] = (int)(theta2*180.0/PI);
	
	return 0;
}
