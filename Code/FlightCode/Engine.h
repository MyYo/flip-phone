#ifndef _ENGINE_H_
#define _ENGINE_H_

void   Eng_Init ();   //Initializes Hardware
void   Eng_StartForward ();   //Make sure you call this function once, otherwise the motor will start-break cycle.
void   Eng_StartBackward ();  //Make sure you call this function once, otherwise the motor will start-break cycle.
void   Eng_Break ();          //Make sure you call this function once, otherwise the motor will start-break cycle.

void   Eng_Test (); //Tester function

#endif
