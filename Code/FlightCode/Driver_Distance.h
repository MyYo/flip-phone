#ifndef _DRIVER_DISTANCE_H
#define _DRIVER_DISTANCE_H

#define UP_FACING_PING 1
#define DOWN_FACING_PING -1
#define NO_DEVICE_SELECTED 0 //AKA disabled

void   Dist_Init ();   //Initializes Hardware
void   Dist_SetActiveDevice(int whichPingDeviceToSet); 
void   Dist_Measure(); //Measure distance 
float  Dist_GetDistance(); //[m]
void   Dist_ExportData(float &whichPing, float &currDist); //Export data for logging

void   Dist_Test (); //Tester function


#endif