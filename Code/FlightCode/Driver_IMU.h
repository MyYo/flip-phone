#ifndef _DRIVER_IMU_H_
#define _DRIVER_IMU_H_
//This is higher level driver wrapper of Driver_IMU_MPU9250

#include "Driver_IMU_MPU9250.h"

void   IMU_Init ();   //Initializes Hardware
void   IMU_Measure(); //Update IMU Data with current measurments
float  IMU_GetAccMag (); //Return current acceleration magnitude (g)
void   IMU_GetOrientation  (float &q1,float &q2,float &q3, float &q4); //Return current orientation IF->BF
float  IMU_GetZenitAngle (); //Returns current angle to zenit (deg)
void   IMU_GetRotationRate (float &omegaX,float &omegaY,float &omegaZ); //Return current rotation rate [rad/sec]
void   IMU_ExportData(float dataArray[]); //Export data for logging

void   IMU_Test (); //Tester function
#endif