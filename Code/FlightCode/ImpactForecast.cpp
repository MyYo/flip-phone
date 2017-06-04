#include "ImpactForecast.h"
#include "Logger.h"

float t[MAX_NUMBER_DIST_MEASURMENTS]; //[sec]
float h[MAX_NUMBER_DIST_MEASURMENTS]; //[m]
int numberOfMeasurments=0;

void IMFO_Init ()
{
	numberOfMeasurments=0;
}

void IMFO_AddDataPoint (unsigned long timeMs, float distanceM) //time in [msec], distance in m
{
	if (numberOfMeasurments<MAX_NUMBER_DIST_MEASURMENTS)
  {
	  t[numberOfMeasurments] = float(timeMs)/1000.0;
	  h[numberOfMeasurments] = distanceM;

    numberOfMeasurments++;
  }
}

//Returns time of impact [msec], reutn 0 if error happend
unsigned long IMFO_PredictTimeofImpact ()
{
	float h0, v0;

	//See matlab code IMFO_EquationGenerator.m for the equations below
	switch(numberOfMeasurments)
	{
	case 0:
	case 1:
	case 2:
		Log_AddNote("Need more measurments for IMFO");
		return 0;
      
	case 3:
		Log_AddNote("3 data points received");
		h0 = - (h[0] + (g*t[0]*t[0])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[1] + (g*t[1]*t[1])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[1]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[2] + (g*t[2]*t[2])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[2]*(t[0] + t[1] + t[2]))/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])));
		v0 = - (h[0] + (g*t[0]*t[0])/2)*((3*t[0])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[1] + (g*t[1]*t[1])/2)*((3*t[1])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2]))) - (h[2] + (g*t[2]*t[2])/2)*((3*t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])) - (t[0] + t[1] + t[2])/(2*(t[0]*t[1] + t[0]*t[2] + t[1]*t[2] - t[0]*t[0] - t[1]*t[1] - t[2]*t[2])));
		break;

	case 4:
	default: //If more samples are found, use the formula for the maximum number of samples
		Log_AddNote("4 data points received");
		h0 = - (h[0] + (g*t[0]*t[0])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[1] + (g*t[1]*t[1])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[1]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[2] + (g*t[2]*t[2])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[2]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3])) - (h[3] + (g*t[3]*t[3])/2)*((t[0]*t[0] + t[1]*t[1] + t[2]*t[2] + t[3]*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[3]*(t[0] + t[1] + t[2] + t[3]))/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]));
		v0 = - ((4*t[0])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[0] + (g*t[0]*t[0])/2) - ((4*t[1])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[1] + (g*t[1]*t[1])/2) - ((4*t[2])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[2] + (g*t[2]*t[2])/2) - ((4*t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]) - (t[0] + t[1] + t[2] + t[3])/(2*t[0]*t[1] + 2*t[0]*t[2] + 2*t[0]*t[3] + 2*t[1]*t[2] + 2*t[1]*t[3] + 2*t[2]*t[3] - 3*t[0]*t[0] - 3*t[1]*t[1] - 3*t[2]*t[2] - 3*t[3]*t[3]))*(h[3] + (g*t[3]*t[3])/2);
		break; 
		
		//TBD Add more cases until 7 points
	}

	Log_AddNote("h0=" + String(h0) + "[m]; v0=" + String(v0)  + "[m/sec]");
	unsigned long tImp = (unsigned long)(1000*(v0 + sqrt(v0*v0 + 2*g*h0))/g); //tImp in Msec
  
	return tImp;
}

unsigned long IMFO_WhenToStartEngine (unsigned predictedImpactTimeMs, float predictedZenitAngle) //Returns time to start engine, based on impact time and impact orientation
{
	//TBD, do the magic
 
	return 0;
}

void IMFO_Test_ImpactTime()
{
	//TBD - make sure prediction fits measurments and record run time
	Log_Init();
	Log_DefineNextField("NSamples","n/a"); 
	Log_DefineNextField("ImpactTime","msec"); 
	Log_WriteLogHeader();
	
	for (int nMs=0;nMs<=7;nMs++)
	{
		int nM=nMs; //How many measurments to test

		IMFO_Init ();

		Log_SetTime(millis());
		Log_SetData(0,nM);

		if (nM>0) {IMFO_AddDataPoint(20, 0.996); nM--;}
		if (nM>0) {IMFO_AddDataPoint(40, 0.988); nM--;}
		if (nM>0) {IMFO_AddDataPoint(60, 0.976); nM--;}
		if (nM>0) {IMFO_AddDataPoint(80, 0.961); nM--;}
		if (nM>0) {IMFO_AddDataPoint(100,0.941); nM--;}
		if (nM>0) {IMFO_AddDataPoint(120,0.917); nM--;}
		if (nM>0) {IMFO_AddDataPoint(140,0.890); nM--;}
	
		long impactTime = IMFO_PredictTimeofImpact();
		Log_SetData(1,impactTime);
		Log_WriteLine();
	}
	Log_Close();

	//Acording to IMFO_GenerateTestVector.m, results should be:
	//NSamples	h0[m]	v0[m]	tImp[msec]
	//3			1.0001	-0.1076	440
	//4			0.9997	-0.0945	441
	//5			0.9998	-0.0964	441
	//6			1.0000	-0.1004	441
	//7			0.9999	-0.0991	441  
}

void IMFO_Test()
{
	//Uncomment to choose to test impact time or when to start engine
	IMFO_Test_ImpactTime();
}
	
