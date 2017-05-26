#include "HardwareConfiguration.h"
#include "Logger.h"
#include "wire.h"

#ifdef LOG_TO_SD
//Include SD Card dirver only if SD card is used
#include "Driver_SD_Card.h" 
#endif 

#define N_OF_LOG_FILEDS 20 //N-1 fields + logic state


float lineData[N_OF_LOG_FILEDS];
String lineNote;
String  fNames[N_OF_LOG_FILEDS-1];
String  fUnits[N_OF_LOG_FILEDS-1];

//Deletes all data from line
void clearLine ()
{
	for (int i=0;i<N_OF_LOG_FILEDS;i++)
	{
		lineData[i]=0;
	}
	lineNote = "";
}

void Log_Init ()
{
#ifdef LOG_TO_SD
	SD_Init();
#else
	Serial.begin(9600);
	Wire.begin();
#endif

	//Initiate filed
	for (int i=0;i<N_OF_LOG_FILEDS;i++)
	{
		fNames[i]="";
		fUnits[i]="";
	}
	clearLine();
}

void logWrite(String message)
{
#ifdef LOG_TO_SD
	SD_Log(message);
#else
	Serial.println(message);
#endif
}

void Log_DefineField (int fI,String fName, String fUnit)
{
	if (fI >= (N_OF_LOG_FILEDS-1))
		logWrite("Too many loged fields, consider increasing N_OF_LOG_FILEDS");
	else
	{
		fNames[fI] = fName;
		fUnits[fI] = fUnit;
	}
}
void Log_SetData (int fI, float data) {lineData[fI] = data;}
void Log_SetLoigcState(int newState){lineData[N_OF_LOG_FILEDS-1] = newState;}
void Log_AddNote(String note) {lineNote += note + ".";}

void Log_WriteLogHeader () //Write log header
{
	//Header
	String message = "";
	for (int i=0;i<N_OF_LOG_FILEDS-1;i++)
		message = message+fNames[i] + " [" + fUnits[i] + "],";
	message += "LogicState,Notes";
	logWrite(message);
}

void Log_WriteLine ()
{
	String message = "";
	for (int i=0;i<N_OF_LOG_FILEDS;i++)
		message += String(lineData[i]) + ",";

	message += lineNote;
	logWrite(message);
	clearLine();
}

//This function implements the IMU tester configuration
//The idea is to print a demo log
void Log_Test ()
{
	Log_Init();

	logWrite("Hello World!");

	for (int i=0;i<N_OF_LOG_FILEDS-1;i++)
		Log_DefineField(i,"A","m/sec");

	Log_WriteLogHeader();

	for (int i=0;i<N_OF_LOG_FILEDS-1;i++)
		Log_SetData(i,i*2.0);

  Log_AddNote("Note");
	Log_WriteLine();

	//TBD: Run and see the log file is ok!
}
