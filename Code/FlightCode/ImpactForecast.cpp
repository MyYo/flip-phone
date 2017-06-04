#include "ImpactForecast.h"
#include "Logger.h"

float t[MAX_NUMBER_DIST_MEASURMENTS]; //[sec]
float h[MAX_NUMBER_DIST_MEASURMENTS]; //[m]
int numberOfMeasurments=0;

void IMFO_AddDataPoint (unsigned long timeMs, float distanceM) //time in [msec], distance in m
{
	if (numberOfMeasurments<MAX_NUMBER_DIST_MEASURMENTS)
  {
	  t[numberOfMeasurments] = float(timeMs)*1000.0;
	  h[numberOfMeasurments] = distanceM;

    numberOfMeasurments++;
  }
}

//Returns time of impact [msec], reutn 0 if error happend
unsigned long IMFO_PredictTimeofImpact ()
{
  float h0, v0;
  const float g=9.81; //[m/sec]

  //See matlab code IMFO_EquationGenerator for the equations below
  switch(numberOfMeasurments)
  {
    case 0:
    case 1:
    case 2:
      Log_AddNote("Need more measurments for IMFO");
      return 0;
      
    case 3:
      Log_AddNote("3 data points received");
      h0 = - (h[0] - (g*t[0]*t[0])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[1] - (g*t[1]*t[1])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[1]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[2] - (g*t[2]*t[2])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[2]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])));
      v0 = - (h[0] - (g*t[0]*t[0])/2)*((3*t[0])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[1] - (g*t[1]*t[1])/2)*((3*t[1])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[2] - (g*t[2]*t[2])/2)*((3*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])));
      break;

    case 4:
      Log_AddNote("4 data points received");
      h0 = - (h[0] - (g*t[0]*t[0])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[1] - (g*t[1]*t[1])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[1]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[2] - (g*t[2]*t[2])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[2]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[3] - (g*t[3]*t[3])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[3]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]));
      v0 = - ((4*t[0])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[0] - (g*t[0]*t[0])/2) - ((4*t[1])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[1] - (g*t[1]*t[1])/2) - ((4*t[2])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[2] - (g*t[2]*t[2])/2) - ((4*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[3] - (g*t[3]*t[3])/2);
      break; 
  }
  Log_AddNote(String(h0) + "," + String(v0));
  

  //We have enough data, so forecast!
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

  IMFO_AddDataPoint(10,0.9985);
  IMFO_AddDataPoint(20,0.9983);
  IMFO_AddDataPoint(30,0.9981);
  long a = IMFO_PredictTimeofImpact();
  Log_WriteLine();
  
}
