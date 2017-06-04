#ifndef _IMPACT_FORECAST_H_
#define _IMPACT_FORECAST_H_

#define MAX_NUMBER_DIST_MEASURMENTS 10

//Physical constants used for forcast
const float g=9.81; //[m/sec^2]


void IMFO_Init ();
void IMFO_AddDataPoint (unsigned long timeMs, float distanceM); //time in [msec], distance in m
unsigned long IMFO_PredictTimeofImpact (); //Returns time of impact [msec], reutn 0 if error happend

unsigned long IMFO_WhenToStartEngine (unsigned predictedImpactTimeMs, float predictedZenitAngle); //Returns time to start engine, based on impact time and impact orientation

void IMFO_Test();
#endif
