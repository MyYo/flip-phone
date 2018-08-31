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

// Private functions for predictor main function
void ComputeB(
	//Inputs
	bool  isMeasurmentValid[], //for each measurment in dataset is it valid or not 
	float h0, float v0, //[m],[m/sec] for f correction, set h0=-1 to ignore f correction
	//Outputs
	float *b, //b vector
	float &tAvg, float &bAvg //averages
)
{
	const float g_2 = 1.0 / 2.0*g;
	const float c = 343; //[m/sec] Speed of sound
	float f;

	int N = 0;
	tAvg = 0;
	bAvg = 0;
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		if (!isMeasurmentValid[i])
			continue; //Skip this measurment, its invalid

		if (h0 == -1)
		{
			b[i] = h[i];
		}
		else
		{	
			f = 1 + (v0 - g*t[i]) / c - g*(h0 + v0*t[i] - g_2*t[i] * t[i]) / (c*c);
			b[i] = h[i] / f;
		}
		b[i] += g_2*t[i]*t[i];

		tAvg += t[i];
		bAvg += b[i];
		N++;
	}
	tAvg = tAvg / ((float)N);
	bAvg = bAvg / ((float)N);
}

void SortArray(float *a, int n, float &median) //a - output/input array, size n
{
	for (int i = 1; i < n; ++i)
	{
		float j = a[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < a[k]); k--)
		{
			a[k + 1] = a[k];
		}
		a[k + 1] = j;
	}

	//Serial.println("SORTED");
	//for (int i=0;i<n;i++)
	//	Serial.println(a[i]);

	median = a[n / 2];
}

void FlagOutliers(
	//Inputs
	float b[], //b vector
	//Outputs
	bool  *isMeasurmentValid, //for each measurment in dataset is it valid or not 
	float &h0, float &v0 //Median estimates of h0, v0 [m],[m/sec]
	)
{
	float h0vec[MAX_NUMBER_DIST_MEASURMENTS*(MAX_NUMBER_DIST_MEASURMENTS - 1) / 2];
	float v0vec[MAX_NUMBER_DIST_MEASURMENTS*(MAX_NUMBER_DIST_MEASURMENTS - 1) / 2];

	//Compute all possible pairs
	int i, j; int n = 0;
	for (i = 0; i < numberOfMeasurments; i++)
	{
		for (j = i + 1; j < numberOfMeasurments; j++)
		{
			v0vec[n] = (b[i] - b[j]) / (t[i] - t[j]);
			h0vec[n] = (t[i] * b[j] - b[i] * t[j]) / (t[i] - t[j]);
			
			//Serial.println(v0vec[n]);

			n++;
		}
	}

	//Get Median
	SortArray(h0vec, n, h0);
	SortArray(v0vec, n, v0);
	
	//Weed Out Outliers
	float th = 15e-3; //[m]
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		if (h0 + v0*t[i] - b[i] > th || h0 + v0*t[i] - b[i] < -th)
			isMeasurmentValid[i] = false;
	}
}

//Returns time of impact [msec], reutn 0 if error happend
unsigned long IMFO_PredictTimeofImpact ()
{
	float h0, v0;

	if (numberOfMeasurments <= 4) 
	{
		Log_AddNote("Number of measurments too low. See Impact Time Estimator Implementation Presentation");
		return 0;
	}

	//Define parameters for solver & Init
	float b[MAX_NUMBER_DIST_MEASURMENTS]; //[m]
	bool isMeasurmentValid[MAX_NUMBER_DIST_MEASURMENTS]; //Unitless
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		isMeasurmentValid[i] = true;
	}
	
	//Preprocessing
	float tAvg;
	float bAvg;
	ComputeB(isMeasurmentValid, -1, -1, b, tAvg, bAvg);

	//Remove Outliers
	FlagOutliers(b, isMeasurmentValid,h0,v0);
	
	//Compute Again, this time correct for f
	ComputeB(isMeasurmentValid, h0, v0, b, tAvg, bAvg);
	
	//Compute v0,h0 from linear regression
	float s1 = 0;
	float s2 = 0;
	for (int i = 0; i < numberOfMeasurments; i++)
	{ 
		if (!isMeasurmentValid[i])
			continue;

		s1 += (t[i] - tAvg)*(b[i] - bAvg);
		s2 += (t[i] - tAvg)*(t[i] - tAvg);
	}
	v0 = s1 / s2;
	h0 = bAvg - v0*tAvg;

	//Check residuals
	float e2 = 0; //RMS error
	const float eThreshold = 5e-3;//[m]
	float diff;
	int N = 0;
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		if (!isMeasurmentValid[i])
			continue;

		diff = (b[i]) - (h0 + v0*t[i]);
		e2 += diff*diff;
		N++;
	}
	e2 = e2 / (float(N));

	if (N < 3)
	{
		Log_AddNote("Number of good measurments too low");
		return 0;
	}

	if (e2 > eThreshold*eThreshold)
	{
		Log_AddNote("Residual error above threshold");
		return 0; //RMS higher then thershold 
	}

	Log_AddNote("h0=" + String(h0,3) + "[m]; v0=" + String(v0,3)  + "[m/sec]");
	unsigned long tImp = (unsigned long)(1000*(v0 + sqrt(v0*v0 + 2*g*h0))/g); //tImp in Msec
  
	return tImp;
}

void IMFO_Test_ImpactTime_LowLevel()
{
	//Test Sort & Median Function
	/////////////////////////////
	Serial.println("Test Sort & Median Function");
	float arr[30]; arr[0] = 3; arr[1] = 1; arr[2] = 2; arr[3] = 3; arr[4] = 1;
	float median;

	//Test Numerical Correctness
	SortArray(arr, 5, median);
	if (median != 2)
	{
		Serial.print("Median Error: Median should be 2, instead is ");
		Serial.println(median);
	}
	else
		Serial.println("Median Works");

	//Test run time
	for (int i = 0; i < 30; i++)
	{
		arr[i] = (10-i) % 7;
	}
	unsigned long time = micros();
	SortArray(arr, 30, median);
	time = micros() - time;
	Serial.print("Time to median 30 samples (microsec): ");
	Serial.println(time);

	//Test ComputeB
	///////////////
	Serial.println("Testing ComputeB");
	float b[30];
	bool isMeasurmentValid[30];
	float tAvg, bAvg;

	for (int i = 0; i < 30; i++)
		isMeasurmentValid[i] = true;

	//No h0,v0
	Serial.println("No v0,h0");
	IMFO_Init();
	IMFO_AddDataPoint(0, 1.50000);
	IMFO_AddDataPoint(60, 1.36234);
	IMFO_AddDataPoint(120, 1.18937);
	IMFO_AddDataPoint(180, 0.98108);
	IMFO_AddDataPoint(240, 0.73747);
	IMFO_AddDataPoint(300, 0.45855);
	ComputeB(isMeasurmentValid, -1, 0, b, tAvg, bAvg);	
	Serial.print("                  b is: ");
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		Serial.print(b[i],4);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Expected results for b: 1.5000, 1.3800, 1.2600, 1.1400, 1.0200, 0.9000,");
	Serial.print("tAvg: "); Serial.println(tAvg); 
	Serial.print("bAvg: "); Serial.println(bAvg);

	//No h0,v0
	Serial.println("With v0,h0");
	IMFO_Init();
	IMFO_AddDataPoint(0, 1.49107);
	IMFO_AddDataPoint(60, 1.35191);
	IMFO_AddDataPoint(120, 1.17823);
	IMFO_AddDataPoint(180, 0.97023);
	IMFO_AddDataPoint(240, 0.72806);
	IMFO_AddDataPoint(300, 0.45192);
	ComputeB(isMeasurmentValid, 1.5, -2, b, tAvg, bAvg);
	Serial.print("                  b is: ");
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		Serial.print(b[i], 4);
		Serial.print(", ");
	}
	Serial.println();
	Serial.println("Expected results for b: 1.5000, 1.3800, 1.2600, 1.1400, 1.0200, 0.9000,");

	//Outlier Detection
	///////////////////
	float h0, v0;
	Serial.println("Outlier Detection Function");
	IMFO_Init();
	IMFO_AddDataPoint(0, 1.50000);
	IMFO_AddDataPoint(60, 1.36234);
	IMFO_AddDataPoint(120, 0.62824);
	IMFO_AddDataPoint(180, 0.98108);
	IMFO_AddDataPoint(240, 0.73747);
	IMFO_AddDataPoint(300, 0.45855);
	ComputeB(isMeasurmentValid, -1, 0, b, tAvg, bAvg);
	FlagOutliers(b, isMeasurmentValid, h0, v0);
	Serial.print("                     isMeasurmentValid: ");
	for (int i = 0; i < numberOfMeasurments; i++)
	{
		Serial.print(isMeasurmentValid[i]);
		Serial.print(", ");
	}
	Serial.print("h0 = ");
	Serial.print(h0,4);
	Serial.print(", v0 = ");
	Serial.print(v0,4);
	Serial.println();
	Serial.println("Expected results for isMeasurmentValid: 1, 1, 0, 1, 1, 1, h0 = 1.5000, v0 = -2.0000");
}

void IMFO_Test_ImpactTime()
{
	Log_Init();

	IMFO_Test_ImpactTime_LowLevel();

	Log_DefineNextField("NSamples","n/a"); 
	Log_DefineNextField("ImpactTime","msec"); 
	Log_DefineNextField("CalcTime", "micros");
	Log_WriteLogHeader();
	
	for (int nMs=0;nMs<=8;nMs++)
	{
		int nM=nMs; //How many measurments to test

		IMFO_Init ();

		Log_SetTime(millis());
		Log_SetData(0,nM);

		if (nM>0) { IMFO_AddDataPoint(0.06, 1.1862000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(10.74, 1.1552000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(21.24, 1.1514000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(32.75, 1.1366000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(43.14, 1.1310000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(53.51, 1.1279000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(63.85, 1.1069000); nM--; }
		if (nM>0) { IMFO_AddDataPoint(74.07, 1.1038000); nM--; }
	
		unsigned long computTime = micros();
		long impactTime = IMFO_PredictTimeofImpact();
		computTime = micros() - computTime;
		Log_SetData(1, impactTime);
		Log_SetData(2, computTime);

		Log_WriteLine();
	}
	Log_Close();  

	// Test Vector Generator: Script3_1.m (trail 2). If all measurments above are included:
	//h0 = 1.1618[m], v0 = -0.4054[m / sec], impactTime: 447.108[msec] according to Alg
}

///////// When to Start Motor Utilities

//Utility function for IMFO_WHENTOStartMotor
float x_from_b(float b)
{
	const float e = 2.71828182845905;
	float s2b = sqrt(2 * b);
	return s2b + 1 - (s2b - b) / (1 - pow(e, -s2b));
}

#include "Driver_Motor.h"
unsigned long IMFO_WhenToStartMotor(unsigned predictedImpactTimeMs, float predictedZenitAngle) //Returns time to start motor, based on impact time and impact orientation. predictedZenitAngle is in [deg]
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
	
