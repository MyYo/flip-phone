#ifndef _LOGIC_H_ 
#define _LOGIC_H_

//Logic States
////////////////////////////////////
#define LS_BOOT_UP				0
#define LS_STAND_BY				1
#define LS_DISTANCE_AQUISITION	2
#define LS_IMPACT_FORECAST		3
#define LS_ENGINE_START			4   
#define LS_ENGINE_SHUTDOWN		5
#define LS_IMPACT				6
#define LS_ERROR				7


//Thresholds and Times
////////////////////////////////////
//			Name			  Value //[Units]
const float freefallGThresh = 0;	//[mg] What is the acceleration threshold beyond which we say that we are in freefall?
const float aquisitionTime  = 150;  //[msec]
const float TBD = 0; //TBD: remove this, it is just a place holder

void RunLogic ();
int lsBootUp(int prevLogicState);
int lsStandBy(int prevLogicState);
int lsDistanceAquisition(int prevLogicState);
int lsImpactForecast(int prevLogicState);
int lsEngineStart(int prevLogicState);
int lsEngineShutdown(int prevLogicState);
int lsImpact (int prevLogicState);
int lsError(int prevLogicState);

#endif