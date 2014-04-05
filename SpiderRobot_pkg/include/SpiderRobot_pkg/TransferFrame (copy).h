
float* TransferFrame(short int Mode,short int Leg, float BasePoints[]);
void MultiplyMat(float A[4][4], float B[4][1], float C[4][1]);//,int N, int L, int M);

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
,{    -.5, .866025, 0,        0}
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
