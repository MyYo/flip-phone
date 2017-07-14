#include "orientationPropagate.h"
#include "quaternionFilters.h"
#include "logger.h"

unsigned long t0;
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
	int i = (int)(1.0*(float)(t-t0)/float(OR_PROP_STEP));

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
}
