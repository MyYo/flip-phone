#ifndef _OR_PROP_H_
#define _OR_PROP_H_

#define OR_PROP_STEP 10.0 //msec
#define OR_PROP_TABLE_SIZE 1 //N instances //TBD: set to 60 before compiling

void OrProp_SetInitialConditions (unsigned long tStart, float qt1, float qt2, float qt3, float qt4, float omegaX, float omegaY, float omegaZ); //Initialize propagation
void OrProp_Prop (int howManySteps); //How many table instances to enter in the propagation step
float OrProp_GetZenitAngle (unsigned long t); //Get Zenit angle at specific time [deg]

void Or_Prop_Test (); //Tester
#endif
