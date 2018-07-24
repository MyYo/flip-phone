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
	const int N = numberOfMeasurments;

	if (N < 3)
	{
		Log_AddNote("Number of measurments too low");
		return 0;
	}

	//Solve using linear regression
	float b[MAX_NUMBER_DIST_MEASURMENTS]; //[m]

	//Compute averages
	float tAvg = 0;
	float bAvg = 0;
	for (int i = 0; i < N; i++)
	{
		b[i] = h[i] + 1 / 2 * g*t[i];

		tAvg += t[i];
		bAvg += b[i];
	}
	tAvg = tAvg / ((float)N);
	bAvg = bAvg / ((float)N);

	//Compute v0,h0 from linear regression
	float s1 = 0;
	float s2 = 0;
	for (int i = 0; i < N; i++)
	{ 
		s1 += (t[i] - tAvg)*(b[i] - bAvg);
		s2 += (t[i] - tAvg)*(t[i] - tAvg);
	}
	v0 = s1 / s2;
	h0 = bAvg - v0*tAvg;

	//Check residuals
	float e2 = 0; //RMS error
	const float eThreshold = 5e-3;//[m]
	float diff;
	for (int i = 0; i < N; i++)
	{
		diff = (h[i]) - (h0 + v0*t[i] - 1 / 2 * g * t[i]*t[i]);
		e2 += diff*diff;
	}
	e2 = e2 / (float(N));

	if (e2 > eThreshold*eThreshold)
	{
		Log_AddNote("Residual error above threshold");
		return 0; //RMS higher then thershold 
	}

	Log_AddNote("h0=" + String(h0) + "[m]; v0=" + String(v0)  + "[m/sec]");
	unsigned long tImp = (unsigned long)(1000*(v0 + sqrt(v0*v0 + 2*g*h0))/g); //tImp in Msec
  
	return tImp;
}

//Utility function for IMFO_WHENTOStartMotor
float x_from_b(float b)
{
	const float e = 2.71828182845905;
	float s2b = sqrt(2 * b);
	return s2b + 1 - (s2b - b) / (1 - pow(e, -s2b));
}

#include "Driver_Motor.h"
unsigned long IMFO_WhenToStartMotor (unsigned predictedImpactTimeMs, float predictedZenitAngle) //Returns time to start motor, based on impact time and impact orientation. predictedZenitAngle is in [deg]
{
	return predictedImpactTimeMs;

	//Measured constants, see presentation for details
	const float E = 266.9; //[rad/sec]
	float Vin = Motor_MeasureMotorDriverInputVoltage(); //[V]. Default value 5.75
	const float tc = 0.3235; //[sec]
	const float r = 20; //Unitless

	float theta = abs(predictedZenitAngle) * PI / 180;
	float b = theta*r / (tc*E*Vin);
 
	float t = x_from_b(b)*tc;
	long  tMsec = t*1000.0;
	
	return predictedImpactTimeMs - tMsec;
  
	return 0; //Returns error
}

void IMFO_Test_ImpactTime()
{
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

void IMFO_Test_WhenToStartMotor()
{
	Log_Init();
	Log_DefineNextField("Case", "n/a");
	Log_WriteLogHeader();

	//Test x_from_b
	float x, xGT;
	int i = 0;

	Log_SetTime(millis());
	Log_SetData(0, i); i++;
	x = x_from_b(0.01478); xGT = 0.17698;
	if (abs(x - xGT) * 314 < 2.0)
		Log_AddNote("Pass");
  Log_WriteLine();
		
	Log_SetTime(millis());
	Log_SetData(0, i); i++;
	x = x_from_b(0.05586); xGT = 0.35395;
	if (abs(x - xGT) * 314 < 2.0)
		Log_AddNote("Pass");
  Log_WriteLine();

	Log_SetTime(millis());
	Log_SetData(0, i); i++;
	x = x_from_b(0.11899); xGT = 0.53093;
	if (abs(x - xGT) * 314 < 2.0)
		Log_AddNote("Pass");
  Log_WriteLine();
  
	Log_SetTime(millis());
	Log_SetData(0, i); i++;
	x = x_from_b(0.20058); xGT = 0.70791;
	if (abs(x - xGT) * 314 < 2.0)
		Log_AddNote("Pass");
  Log_WriteLine();
  
	Log_SetTime(millis());
	Log_SetData(0, i); i++;
	x = x_from_b(0.29765); xGT = 0.88488;
	if (abs(x - xGT) * 314 < 2.0)
		Log_AddNote("Pass");
  Log_WriteLine();
  
	Log_Close();
}

void IMFO_Test()
{
	//Uncomment to choose to test impact time or when to start motor
	IMFO_Test_ImpactTime();
	IMFO_Test_WhenToStartMotor();
}
	
