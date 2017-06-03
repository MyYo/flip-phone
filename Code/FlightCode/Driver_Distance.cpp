#include "HardwareConfiguration.h"
#include "Driver_Distance.h"
#include "Arduino.h"

//Based on Ping Driver
int whichPingDevice;
float currentDistance; //m

//Initializes Hardware
void   Dist_Init ()
{
	whichPingDevice = NO_DEVICE_SELECTED;
}

void Dist_SetActiveDevice(int whichPingDeviceToSet)
{
	whichPingDevice = whichPingDeviceToSet;
}

//Measure distance [m]
void  Dist_Measure()
{
	int pinNumber;
	if (whichPingDevice)
		pinNumber = PIN_PING_DOWNFACING;
	else
		pinNumber = PIN_PING_UPFACING;
	
	long pulseDuration=0;
	// set pin as output so we can send a pulse
	pinMode(pinNumber, OUTPUT);
	// set output to LOW
	digitalWrite(pinNumber, LOW);
	delayMicroseconds(5);
 
	// now send the 5uS pulse out to activate Ping)))
	digitalWrite(pinNumber, HIGH);
	delayMicroseconds(5);
	digitalWrite(pinNumber, LOW);
 
	// now we need to change the digital pin
	// to input to read the incoming pulse
	pinMode(pinNumber, INPUT);
 
	// finally, measure the length of the incoming pulse
	pulseDuration=pulseIn(pinNumber, HIGH);
	// divide the pulse length by half
	pulseDuration=pulseDuration/2; 
 
	// now convert to meters. We're metric here people...
	currentDistance = int((pulseDuration/29.0)/100.0);
}
float  Dist_GetDistance() {return currentDistance;}
void   Dist_ExportData(float dataArray[]) //Export data for logging
{
	dataArray[0] = whichPingDevice;
	dataArray[1] = currentDistance;
}

//This function implements the Dist tester configuration
//The idea is we will run the Dist_Test function instead of the logic, it will spit out data while we change distance and see data matches our estimates
#include "Logger.h"
void Dist_TestLogInit ()
{
  //Initiate
  Log_Init();
  Log_DefineNextField("Ping_Dev","N/A");
  Log_DefineNextField("Dist","m");

  Log_WriteLogHeader();
}

//Tester function
void   Dist_Test ()
{
  Disg_TestLogInit();
  Dist_Init();
  Dist_SetActiveDevice(DOWN_FACING_PING);
  
  float dataArray[2];
  while (true) //Loop forever
   {
    //Read dist and log values
    Log_SetTime(millis());
    Dist_Measure();
    
    Dist_ExportData(dataArray);
    for(int i=0;i<2;i++)
      Log_SetData(i,dataArray[i]);
  
    //Log
    Log_WriteLine();
   }

   //TBD - Check to see if it works
 
}
