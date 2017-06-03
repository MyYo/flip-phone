#include "ImpactForecast.h"

unsigned long  time[MAX_NUMBER_DIST_MEASURMENTS]; //[msec]
float dist[MAX_NUMBER_DIST_MEASURMENTS]; //[m]
int numberOfMeasurments=0;

void IMFO_AddDataPoint (unsigned long timeMs, float distanceM) //time in [msec], distance in m
{
	if (numberOfMeasurments<MAX_NUMBER_DIST_MEASURMENTS)
  {
	  time[numberOfMeasurments] = timeMs;
	  dist[numberOfMeasurments] = distanceM;

    numberOfMeasurments++;
  }
}

//Returns time of impact [msec], reutn 0 if error happend
unsigned long IMFO_PredictTimeofImpact ()
{
	//TBD, do you magic
	return 0;
}

unsigned long IMFO_WhenToStartEngine (unsigned predictedImpactTimeMs, float predictedZenitAngle) //Returns time to start engine, based on impact time and impact orientation
{
	//TBD, do the magic
	return 0;
}

void IMFO_Test()
{
	//Add some sample data, and see that the right result comes out.
	//Also measure how long it took to execute
}
