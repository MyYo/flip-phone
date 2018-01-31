#include "orientationPropagate.h"
#include "quaternionFilters.h"
#include "logger.h"

unsigned long t0; //[msec]
float zenitAngles[OR_PROP_TABLE_SIZE];  //[deg]

//Save current step of propagation
int currentTableIndex;
float currentQt[4];
float currentOmega[3];

void OrProp_SetInitialConditions (unsigned long tStart, float qt1, float qt2, float qt3, float qt4, float omegaX, float omegaY, float omegaZ)
	//Initialize propagation
{
	currentTableIndex = 0;
	currentQt[0]  = qt1;
	currentQt[1]  = qt2;
	currentQt[2]  = qt3;
	currentQt[3]  = qt4;
	currentOmega[0] = omegaX;
	currentOmega[1] = omegaY;
	currentOmega[2] = omegaZ;

	zenitAngles[currentTableIndex] = QtAngleToZenit (currentQt[0],currentQt[1],currentQt[2],currentQt[3]);

	t0 = tStart;
}

void OrProp_Prop (int howManySteps) //How many table instances to enter in the propagation step
{
	for (int j=0;j<howManySteps;j++)
	{
		currentTableIndex++;
		if (currentTableIndex>OR_PROP_TABLE_SIZE)
		{
			//No more room in propagation table, we are done
			return;
		}


		//TBD integrate equations
		//

		zenitAngles[currentTableIndex] = QtAngleToZenit (currentQt[0],currentQt[1],currentQt[2],currentQt[3]);
	}
}
float OrProp_GetZenitAngle (unsigned long t) //Get Zenit angle at specific time [deg]
{
	float dt = (float)t - (float)t0;

	int i = (int)(dt / OR_PROP_STEP); //Find Nearest Neigbor index of Zenit Angle from list

	if (i>=currentTableIndex)
	{
		//Warning, grid is missing for that time point
		Log_AddNote("WARNING: No grid for requested time point");
		return 0;
	}
	else
		return zenitAngles[i];
}

void Or_Prop_Test ()
//Tester
{
	//TBD
	//Do a test for propagation of angles currectly, and timing test - how long does it take to propagate some amount
 //Try to see what the test is in the IMU and do the same

  //Setup logger
  Log_Init();
  Log_DefineNextField("Step Number","N/A");
  Log_DefineNextField("ZenithAngle","deg");
  Log_WriteLogHeader();

 //I think this is euler angles 0,0,0? Do a read of phone's qt and move it such that it is at 1,0,0,0 to validate what orientation it looks like
  float qt1 = 1;
  float qt2 = 0;
  float qt3 = 0;
  float qt4 = 0;
  

  //[rad/sec]
  float omegaX = 0;
  float omegaY = 0;
  float omegaZ = PI/4;

  //Number of steps
  int numSteps = 10;
  OrProp_SetInitialConditions (millis(), qt1, qt2, qt3, qt4, omegaX, omegaY, omegaZ);
  
  OrProp_Prop(numSteps);
  for (int i=0;i<numSteps;i++)
  {
    Log_SetData(0,i);
    Log_SetData(1,zenitAngles[i]);
    Log_WriteLine();
  }
}

