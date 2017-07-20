#include "logic.h"
#include "logger.h"
#include "Driver_IMU.h"
#include "Driver_Distance.h"
#include "OrientationPropagate.h"
#include "ImpactForecast.h"
#include "Engine.h"

//Timers
unsigned long tCurrentTime; //[ms]
unsigned long tFallDetectTime_T0=0; //[ms]
unsigned long tEndOfDataAquisitionTime_T1=0; //[ms]
unsigned long tEngineStartTime=0; //[ms]
unsigned long tTimeOfImpact_T2_Predicted=0; //[ms]
unsigned long tTimeOfImpact_T2_Actual=0; //[ms]

bool enginePolarization; //True - FW, false - BW

void logicGatherData ();

void RunLogic ()
{
	int currentState = LS_BOOT_UP;
	int prevState = LS_BOOT_UP;
	int nextState = LS_BOOT_UP;

	while(true) //Loop forever
	{
		tCurrentTime = millis();
		Log_SetTime(tCurrentTime);

		//Hendle logic
		switch(currentState)
		{
		case LS_BOOT_UP:
			nextState=lsBootUp(prevState);
			break;
		case LS_STAND_BY:
			nextState=lsStandBy(prevState);
			break;
		case LS_DISTANCE_AQUISITION:
			nextState=lsDistanceAquisition(prevState);
			break;
		case LS_IMPACT_FORECAST:
			nextState=lsImpactForecast(prevState);
			break;
		case LS_ENGINE_START:
			nextState=lsEngineStart(prevState);
			break;
		case LS_ENGINE_SHUTDOWN:
			nextState=lsEngineShutdown(prevState);
			break;
		case LS_IMPACT:
			nextState=lsImpact(prevState);
			break;
		case LS_ERROR:
			nextState=lsError(prevState);
			break;
		}

		//Log
		if (
			tFallDetectTime_T0>0 && (tCurrentTime-tFallDetectTime_T0>= 1*1000)  //If fall detected and x time passed since
			|| 
			tCurrentTime>= 30*1000 //Maximum time reached
			)
		{
			//Too long has passed since start of the experiment / fall start. Close the log
			Log_Close();
		}
		else
		{
			logicGatherData();
			Log_SetLoigcState(currentState);
			if (nextState != currentState)
				Log_AddNote("State Changed");
			Log_WriteLine();
		}

		prevState = currentState;
		currentState = nextState;

		//delay(10);
	}
}

void logicGatherData ()
{
	float tmp[20];

	int i=0;

	//IMU Telemetry
	Log_SetData(i,IMU_GetAccMag()); i++;
	Log_SetData(i,IMU_GetZenitAngle()); i++;
	float omega[3];
	IMU_GetRotationRate (omega[0],omega[1],omega[2]);
	for (int j=0;j<3;j++)
		Log_SetData(i,omega[j]); i++;

	//Distance Sensor Telemetry
	Dist_ExportData(tmp);
	Log_SetData(i,tmp[1]); i++;
	Log_SetData(i,tmp[2]); i++;
}

int lsBootUp(int prevLogicState)
{
	Log_Init();
	Log_DefineNextField("AccMag","mg"); 
	Log_DefineNextField("ZenitAng","deg"); 
	Log_DefineNextField("omegaX","rad/sec"); 
	Log_DefineNextField("omegaY","rad/sec"); 
	Log_DefineNextField("omegaZ","rad/sec"); 
	Log_DefineNextField("DistToGND","[m]"); 
	Log_DefineNextField("DistDevice","#");
	Log_WriteLogHeader();

	Dist_Init();
	IMU_Init();
	IMFO_Init();
  Eng_Init();

	return LS_STAND_BY;
}
int lsStandBy(int prevLogicState)
{
	IMU_Measure();

	if (IMU_GetAccMag () < freefallGThresh)
	{
		Log_AddNote("Fall Detected");
		tFallDetectTime_T0 = tCurrentTime;

		//Set initial conditions for orientation propagator
		float q1,q2,q3,q4,omegaX,omegaY,omegaZ;
		IMU_GetOrientation  (q1,q2,q3,q4); //Return current orientation IF->BF
		IMU_GetRotationRate (omegaX,omegaY,omegaZ); //Return current rotation rate
		
		OrProp_SetInitialConditions(tFallDetectTime_T0,q1,q2,q3,q4,omegaX,omegaY,omegaZ);
		OrProp_Prop (2); //Propagate a little to give us some grid to start with before moving to the next step

		return LS_DISTANCE_AQUISITION;
	}
	else
		return LS_STAND_BY;
}

int lsDistanceAquisition(int prevLogicState)
{
	if (tCurrentTime < tFallDetectTime_T0+aquisitionTime)
	{
		//Aquire Data
		float ang = OrProp_GetZenitAngle (tCurrentTime); //[deg]
		if      (abs(ang-180) <= pingHalfFOV || abs(ang+180) <= pingHalfFOV) //Upfacing Ping is installed at angle = 0 [deg] 
			Dist_SetActiveDevice(UP_FACING_PING);
		else if (abs(ang-  0) <= pingHalfFOV || abs(ang-360) <= pingHalfFOV) //Upfacing Ping is installed at angle = 180 [deg]. 360 case is just in case
			Dist_SetActiveDevice(DOWN_FACING_PING);
		else
		{
			//Ground is visible, don't measure
			Dist_SetActiveDevice(NO_DEVICE_SELECTED);

			//Since we don't measure, we can propagate a bit longer
			OrProp_Prop (20);
			return LS_DISTANCE_AQUISITION;
		}

		Dist_Measure(); //Measure distance 
		IMFO_AddDataPoint(tCurrentTime-tFallDetectTime_T0,Dist_GetDistance()); //Log data with respect to fall start

		//Propagate
		OrProp_Prop (10);
		return LS_DISTANCE_AQUISITION;
	}
	else
	{
		//Aqusition Time is Over
		Dist_SetActiveDevice(NO_DEVICE_SELECTED); //Disable distance aquisition

		return LS_IMPACT_FORECAST;
	}
}
int lsImpactForecast(int prevLogicState)
{
	//Compute time of impact
	tTimeOfImpact_T2_Predicted = IMFO_PredictTimeofImpact()+tFallDetectTime_T0;
	Log_AddNote("Predicted T2=" + String(tTimeOfImpact_T2_Predicted) + "[msec]");

	//Compute Angle at time of impact
	float angleAtT2 = OrProp_GetZenitAngle (tTimeOfImpact_T2_Predicted);
	Log_AddNote("Predicted AngT2=" + String(angleAtT2) + "[deg]");

	tEngineStartTime = IMFO_WhenToStartEngine (tTimeOfImpact_T2_Predicted, angleAtT2);
	if (tEngineStartTime == 0)
	{
		//Error happend
		return LS_ERROR;
	}

	if (angleAtT2 < 0)
		enginePolarization = true; //Set Engine forward polarization
	else 
		enginePolarization = false; //Set Engine backward polarization

	return LS_ENGINE_START;
}
int lsEngineStart(int prevLogicState)
{
	if (tCurrentTime < tEngineStartTime)
		//Wait for the rigth time to start engine
		return LS_ENGINE_START;
	else
	{
		//Start Engine!
   
    Log_AddNote("Eng Start");
    if(enginePolarization){
      Eng_StartForward();
    }
    else {
      Eng_StartBackward();
    }
		
		return LS_ENGINE_SHUTDOWN;
	}
}
int lsEngineShutdown(int prevLogicState)
{
	if (tCurrentTime < tTimeOfImpact_T2_Predicted)
	{
		//Waiting for impact
		IMU_Measure();
		if (IMU_GetAccMag() > restoredGThresh) //[g]
		{
			//Impact Detected
			Eng_Break(); //Stop Engine
			Log_AddNote("Eng Break, Impact");
			return LS_IMPACT;
		}
		else
			return LS_ENGINE_SHUTDOWN;
	}
	else 
	{
		//Time expired
		Eng_Break(); //Stop Engine
		Log_AddNote("Eng Break");
		return LS_IMPACT;
	}
}

int lsImpact (int prevLogicState)
{
	IMU_Measure();
	if (IMU_GetAccMag() > restoredGThresh) //[g]
	{
		//Impact Detected
		double ang = IMU_GetZenitAngle(); //[deg]
		tTimeOfImpact_T2_Actual = tCurrentTime;

		Log_AddNote("Impact!");
		Log_AddNote("Actual T2=" + String(tTimeOfImpact_T2_Actual) + "[msec]");
		Log_AddNote("Actual AngT2=" + String(ang) + "[deg]");

		return LS_STAND_BY;
	}
	else
	{
		//Wait for Impact
		return LS_IMPACT;
	}
}

int lsError(int prevLogicState)
{
	return LS_STAND_BY;
}
