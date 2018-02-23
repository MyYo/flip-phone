#ifndef _ENGINE_H_
#define _ENGINE_H_

void   Motor_Init ();   //Initializes Hardware
void   Motor_StartForward ();   //Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_StartBackward ();  //Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_Break ();          //Make sure you call this function once, otherwise the motor will start-break cycle.

float Motor_MeasureMotorDriverInputVoltage(); //Returns the motor driver input voltage (capacitor voltage)

void   Motor_Test (); //Tester function

#endif
