#include "logic.h"
#include "logger.h"
#include "Driver_IMU.h"
#include "Driver_Distance.h"
#include "Driver_SafetyPin.h"
#include "Driver_Motor.h"
#include "OrientationPropagate.h"
#include "ImpactForecast.h"
#include "HardwareConfiguration.h"

//Timers
unsigned long tCurrentTime; //[ms]
unsigned long tFallDetectTime_T0=0; //[ms]
unsigned long tEndOfDataAquisitionTime_T1=0; //[ms]
unsigned long tMotorStartTime=0; //[ms]
unsigned long tTimeOfImpact_T2_Predicted=0; //[ms]
unsigned long tTimeOfImpact_T2_Actual=0; //[ms]

bool motorPolarization; //True - FW, false - BW

void logicGatherData ();

void RunLogic ()
{
	int currentState = LS_BOOT_UP;
	int prevState = LS_OFF;
	int nextState = LS_BOOT_UP;

	long loopI = 0;
	Log_Init();
	while(true) //Loop forever
	{
		tCurrentTime = millis();
		Log_SetTime(tCurrentTime - tFallDetectTime_T0); //Before fall detection current time = millis(). After, time is measured from t0

		//Hendle logic
		Log_SetLoigcState(currentState);
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
		case LS_WAIT_FOR_ENGINE_START:
			nextState=lsMotorStart(prevState);
			break;
		case LS_WAIT_FOR_ENGINE_SHUTDOWN:
			nextState=lsMotorShutdown(prevState);
			break;
		case LS_IMPACT:
			nextState=lsImpact(prevState);
			break;
		case LS_ERROR:
			nextState=lsError(prevState);
			break;
		case LS_DUMP_DATA:
			nextState = lsDumpData(prevState);
			break;
		}
		
		//Global Change States
		if (SafetyPin_IsConnected() && currentState != LS_BOOT_UP)
		{
			//Safety pin got reconnected, force state change
			nextState = LS_DUMP_DATA;
		}

		//Log
		logicGatherData();
		if (nextState != currentState)
			Log_AddNote("State Changed");
		Log_WriteLine();

		//Change logic state
		prevState = currentState;
		currentState = nextState;

		while (millis() < tCurrentTime + 10); //Each iteration has a minimal size
	
		loopI++;
	}
}

void logicGatherData ()
{
	float tmp[N_OF_LOG_FIELDS];

	int i=0;

	Log_SetData(i, millis()-tCurrentTime); i++; //How long this iteration lasted

	//IMU Telemetry
	Log_SetData(i,IMU_GetAccMag()); i++;
	Log_SetData(i,IMU_GetZenitAngle()); i++;
	float omega[3];
	IMU_GetRotationRate (omega[0],omega[1],omega[2]);
	for (int j = 0; j < 3; j++)
	{
		Log_SetData(i, omega[j]);
		i++;
	}
	
	//Distance Sensor Telemetry
	float whichPing, currDist;
	Dist_ExportData(whichPing, currDist);
	Log_SetData(i, currDist*1000); i++; //Distane converted to mm
	Log_SetData(i, whichPing); i++;

	//Motor/Capacitor Voltage
	float v = Motor_MeasureMotorDriverInputVoltage();
	Log_SetData(i, v); i++;
	if (v > 5.5) 
	{
		//Turn LED on if voltage is ok
		digitalWrite(PIN_LED_VOLTAGE_OK_INDICATOR, HIGH);
	}
	else 
	{
		digitalWrite(PIN_LED_VOLTAGE_OK_INDICATOR, LOW);
	}
	
}

int lsBootUp(int prevLogicState)
{
	if (prevLogicState == LS_OFF)
	{
		//Run only from off state

		Log_Init();
		Log_DefineNextField("IterLen", "msec"); //Iteration length
		Log_DefineNextField("AccMag", "g");
		Log_DefineNextField("ZenitAng", "deg");
		Log_DefineNextField("omegaX", "rad/sec");
		Log_DefineNextField("omegaY", "rad/sec");
		Log_DefineNextField("omegaZ", "rad/sec");
		Log_DefineNextField("DistToGND", "mm");
		Log_DefineNextField("DistDevice", "#");
		Log_DefineNextField("Capacitor", "V");
		Log_WriteLogHeader();

		Dist_Init();
		IMU_Init();
		IMFO_Init();
		Motor_Init();
		SafetyPin_Init();

		//Initiate Capacitor Voltage Indicator
		pinMode(PIN_LED_VOLTAGE_OK_INDICATOR, OUTPUT);
		digitalWrite(PIN_LED_VOLTAGE_OK_INDICATOR, LOW);
		
	}

	//Wait until safety plug is removed
	if (SafetyPin_IsConnected())
	{
		delay(500); //Wait a bit before inquiring again
		return LS_BOOT_UP;
	}
	
	//Wait a bit before changing to the next state
	delay(1000);
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
	if (tCurrentTime < tFallDetectTime_T0+aquisitionDuration)
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
		//Acquisition Time is Over
		tEndOfDataAquisitionTime_T1 = tCurrentTime;
		Dist_SetActiveDevice(NO_DEVICE_SELECTED); //Disable distance aquisition

		return LS_IMPACT_FORECAST;
	}
}
int lsImpactForecast(int prevLogicState)
{
	//Compute time of impact
	tTimeOfImpact_T2_Predicted = IMFO_PredictTimeofImpact() + tFallDetectTime_T0;
	Log_AddNote("Predicted T2-T0=" + String(tTimeOfImpact_T2_Predicted - tFallDetectTime_T0) + "[msec]");

	//Compute Angle at time of impact
	float angleAtT2 = OrProp_GetZenitAngle (tTimeOfImpact_T2_Predicted);
	Log_AddNote("Predicted Ang@T2=" + String(angleAtT2) + "[deg]");

	tMotorStartTime = IMFO_WhenToStartMotor (tTimeOfImpact_T2_Predicted, angleAtT2);
	if (tMotorStartTime == 0)
	{
		//Error Happened
		return LS_ERROR;
	}
	if (tTimeOfImpact_T2_Predicted - tMotorStartTime < minimalMotorActivity)
	{
		//Motor Activation Not Required
		return LS_WAIT_FOR_ENGINE_SHUTDOWN;
	}

	if (angleAtT2 < 0)
		motorPolarization = true; //Set Motor forward polarization
	else 
		motorPolarization = false; //Set Motor backward polarization

	return LS_WAIT_FOR_ENGINE_START;
}
int lsMotorStart(int prevLogicState)
{
	//Wait for motor start

	while (millis() < tMotorStartTime); //Wait for correct timing. Since it is important, do not loop through logic. wait for start
		
	//Start Motor!
	if(motorPolarization)
	{
		Motor_StartForward();
	}
	else 
	{
		Motor_StartBackward();
	}
		
	return LS_WAIT_FOR_ENGINE_SHUTDOWN;
}
int lsMotorShutdown(int prevLogicState)
{
	if (tCurrentTime < tTimeOfImpact_T2_Predicted)
	{
		//Waiting for impact
		IMU_Measure();
		if (IMU_GetAccMag() > restoredGThresh) //[g]
		{
			//Impact Detected
			Motor_Break(); //Stop Motor

			Log_AddNote("Motor Break, Impact");

			//Impact time & angle
			double ang = IMU_GetZenitAngle(); //[deg]
			tTimeOfImpact_T2_Actual = tCurrentTime;
			Log_AddNote(" Actual T2-T0=" + String(tTimeOfImpact_T2_Actual - tFallDetectTime_T0) + "[msec]");
			Log_AddNote(" Actual AngT2=" + String(ang) + "[deg]");

			return LS_IMPACT;
		}
		else
			return LS_WAIT_FOR_ENGINE_SHUTDOWN;
	}
	else 
	{
		//Time expired
		Motor_Break(); //Stop Motor
		Log_AddNote("Motor Break");
		return LS_IMPACT;
	}
}

int lsImpact (int prevLogicState)
{
	IMU_Measure();
	if (IMU_GetAccMag() > restoredGThresh) //[g]
	{
		if (prevLogicState != LS_IMPACT)
		{
			//In case we got here from state which is not the LS_IMPACT then impact time was already recorded
			return LS_STAND_BY;
		}

		//Otherwise, record impact time etc
		//Impact Detected
		double ang = IMU_GetZenitAngle(); //[deg]
		tTimeOfImpact_T2_Actual = tCurrentTime;

		Log_AddNote("Impact!");
		Log_AddNote(" Actual T2-T0=" + String(tTimeOfImpact_T2_Actual - tFallDetectTime_T0) + "[msec]");
		Log_AddNote(" Actual AngT2=" + String(ang) + "[deg]");

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


int lsDumpData(int prevLogicState)
{
	Log_DumpCache();
	delay(1000 * 10);

	return LS_STAND_BY; //Set next state to be standby, if plug is still not removed, logic will override and dump again
}
