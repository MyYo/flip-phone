#ifndef _OR_PROP_H_
#define _OR_PROP_H_

const float OR_PROP_STEP 10.0	 //[msec], how dense the grid is
#define OR_PROP_TABLE_SIZE 60 //N instances. Total propogation duration is OR_PROP_STEP*OR_PROP_TABLE_SIZE

//Initialize propagation
//tStart - is the time associated with the orientation and rotation rate [msec]
void OrProp_SetInitialConditions (unsigned long tStart, float qt1, float qt2, float qt3, float qt4, float omegaX, float omegaY, float omegaZ); 
	
//Propagate orientation by 'howManySteps'. This number is set to reduce computation overload
void OrProp_Prop (int howManySteps); 

//Get Zenit angle at specific time [deg]. t is in [msec]
float OrProp_GetZenitAngle (unsigned long t); 

void Or_Prop_Test (); //Tester
#endif
