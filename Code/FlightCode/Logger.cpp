#include "HardwareConfiguration.h"
#include "Logger.h"
#include "wire.h"

#ifdef LOG_TO_SD
//Include SD Card dirver only if SD card is used
#include "Driver_SD_Card.h" 
#endif 

#define N_OF_LOG_FILEDS 20 //N fields

float lineData[N_OF_LOG_FILEDS];
unsigned short logicState;
unsigned long timems;
String lineNote;
String  fNames;
int nFields;

//Deletes all data from line
void clearLine ()
{
	for (int i=0;i<N_OF_LOG_FILEDS;i++)
	{
		lineData[i]=0;
	}
	lineNote = "";
	nFields = 0;
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
	fNames="";
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

void Log_DefineNextField (String fName, String fUnit)
{
	if (nFields >= (N_OF_LOG_FILEDS))
		logWrite("Too many loged fields, consider increasing N_OF_LOG_FILEDS");
	
	fNames = fName + " [" + fUnit + "],";
	nFields++;
	
}
void Log_SetData (int fI, float data) 
{
	if (fI >= (N_OF_LOG_FILEDS))
		logWrite("Too many loged fields, consider increasing N_OF_LOG_FILEDS");
	else
		lineData[fI] = data;
}
void Log_SetLoigcState(unsigned short newState){logicState = newState;}
void Log_SetTime(unsigned long time){timems= time;}
void Log_AddNote(String note) {lineNote += note + ".";}

void Log_WriteLogHeader () //Write log header
{
	//Header
	String message = "Time[msec]," + fNames + "LogicState,Notes";
	logWrite(message);
}

void Log_WriteLine ()
{
	String message = String(timems);
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
		Log_DefineNextField("A","m/sec");

	Log_WriteLogHeader();

	Log_SetTime(10);
	Log_SetLoigcState(10);
	for (int i=0;i<N_OF_LOG_FILEDS-1;i++)
		Log_SetData(i,i*2.0);

	Log_AddNote("Note");
	Log_WriteLine();

	//TBD: Run and see the log file is ok!
}
