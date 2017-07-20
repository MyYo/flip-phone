#ifndef _LOGIC_H_ 
#define _LOGIC_H_

//Logic States
////////////////////////////////////
#define LS_BOOT_UP              0
#define LS_STAND_BY             1
#define LS_DISTANCE_AQUISITION  2
#define LS_IMPACT_FORECAST      3
#define LS_ENGINE_START         4   
#define LS_ENGINE_SHUTDOWN      5
#define LS_IMPACT               6
#define LS_ERROR                7


//Thresholds and Times, TBD: Update after experimenting with values
////////////////////////////////////
//			    Name			        Value //[Units]
const float freefallGThresh = 0.5;	//[g] What is the acceleration threshold beyond which we say that we are in freefall?
const float restoredGThresh = 1;    //[g] What is the acceleration threshold beyond which we say that we have landed?
const float aquisitionTime  = 150;  //[msec]
const float pingHalfFOV     = 45;   //[deg] Ping was tested to have full FOV of 90 degrees, so Half FOV = 45 degrees.


void RunLogic ();
int lsBootUp(int prevLogicState);
int lsStandBy(int prevLogicState);
int lsDistanceAquisition(int prevLogicState);
int lsImpactForecast(int prevLogicState);
int lsMotorStart(int prevLogicState);
int lsMotorShutdown(int prevLogicState);
int lsImpact (int prevLogicState);
int lsError(int prevLogicState);

#endif
