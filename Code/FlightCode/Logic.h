#ifndef _LOGIC_H_ 
#define _LOGIC_H_

//Logic States
////////////////////////////////////
#define LS_OFF						-1
#define LS_BOOT_UP                   0
#define LS_STAND_BY                  1
#define LS_DISTANCE_AQUISITION       2
#define LS_IMPACT_FORECAST           3
#define LS_WAIT_FOR_ENGINE_START     4
#define LS_WAIT_FOR_ENGINE_SHUTDOWN  5
#define LS_IMPACT                    6
#define LS_ERROR                     7
#define LS_DUMP_DATA				 8


//Thresholds and Times
////////////////////////////////////
//			    Name			        Value //[Units]
const float freefallGThresh      = 0.5;	//[g] What is the acceleration threshold beyond which we say that we are in freefall? TBD
const float restoredGThresh      = 0.5; //[g] What is the acceleration threshold beyond which we say that we have landed? TBD
const float aquisitionDuration   = 150; //[msec]
const float pingHalfFOV          = 90;//36;  //[deg] Ping was tested to have half FOV.
const long  minimalMotorActivity = 30;  //[msec] What is the minimal motor run time, below of which no activation happens
const float minMotorVoltage		 = 5.5; //[V] minimal motor voltage for proper operation


void RunLogic ();
int lsBootUp(int prevLogicState);
int lsStandBy(int prevLogicState);
int lsDistanceAquisition(int prevLogicState);
int lsImpactForecast(int prevLogicState);
int lsMotorStart(int prevLogicState);
int lsMotorShutdown(int prevLogicState);
int lsImpact (int prevLogicState);
int lsError(int prevLogicState);
int lsDumpData(int prevLogicState);

//Testers
void ls_TestMomentOfInertia(); //This test runs the motor starting from the fall
void ls_TestImpactTime();      //This test records distance to flour and impact time


#endif
