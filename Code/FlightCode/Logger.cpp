#include "HardwareConfiguration.h"
#include "Logger.h"
#include "wire.h"

#define N_CHARS_PER_FIELD    12 //'space pad' unused charecters so lines will all 'look nice'
#define N_CHARS_PER_FIELDSTR "%12s"

//If Using Cache:
#ifdef LOG_USING_CACHE
#define LOG_CACHE_SIZE 75 //Number of lines
#define CACHE_LINE_LENGTH ((N_OF_LOG_FIELDS + 2 + 1*2)*N_CHARS_PER_FIELD+30) //3 extra fields are time, logic state and note with equivalent of 2 fields
char cache[LOG_CACHE_SIZE][CACHE_LINE_LENGTH+1]; //Fixed cache size, no dynamic allocation
int cacheHead = 0;
#endif

float lineData[N_OF_LOG_FIELDS];
unsigned short logicState;
unsigned long timems;
String lineNote;

String  headderLine1_Names;
String  headderLine2_Units;
int nFields;

////////////////////////////////////////////////////////////////////////////////////////////////
//LED
////////////////////////////////////////////////////////////////////////////////////////////////

//If Compiling for QDUINOMINI, use LED desplay to indicate logic state
#ifdef ARDUINO_SAMD_FEATHER_M0_EXPRESS
#include <Adafruit_NeoPixel.h>
#define PIN8 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN8);
void LED_Init() {
	Wire.begin();
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'
}
void LED_SetColor(unsigned short Color)
{
	const int dimmingFactor = 2; //% Brightness

								 //Select Color
	int r = 0; int g = 0;  int b = 0;
	switch (Color % 9)
	{
	case 0:		r = 255; g = 100; break; //Orange
	case 1:		r = 255; break; //Red
	case 2:		g = 255; break; //Green
	case 3:		b = 255; break; //Blue
	case 4:		r = 255; g = 050; b = 050; break; //Pink
	case 5:		r = 255; b = 255; break; //Purple
	case 6:		g = 255; b = 150; break; //Cyan
	case 7:		r = 255; g = 255; b = 255; break; //White
	case 8:		r = 255; g = 255; b = 255; break; //Yellow
	}

	//Turn LED On
	strip.setPixelColor(0,
		(r*dimmingFactor) / 100,
		(g*dimmingFactor) / 100,
		(b*dimmingFactor) / 100);
	strip.show();
}
#else
//No LED, No desplay
void LED_Init() {}
void LED_SetColor(unsigned short Color) {};
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
//Utilities, clear line, padding
////////////////////////////////////////////////////////////////////////////////////////////////

//Deletes all data from line
void clearLine()
{
	for (int i = 0; i<N_OF_LOG_FIELDS; i++)
	{
		lineData[i] = 0;
	}
	lineNote = "";
}

//'space pad' unused charecters so lines will all 'look nice'
String padWithSpaces(String str)
{
	char strBuff[N_CHARS_PER_FIELD + 2];
	char outBuff[N_CHARS_PER_FIELD + 2];

	str.toCharArray(strBuff, N_CHARS_PER_FIELD);
	sprintf(outBuff, N_CHARS_PER_FIELDSTR, strBuff);

	return String(outBuff);
}

////////////////////////////////////////////////////////////////////////////////////////////////
//'Low Level' functionalities, init, writing, closing
////////////////////////////////////////////////////////////////////////////////////////////////

void Log_Init()
{
	//Begin Serial
	Serial.begin(9600);

#ifdef LOG_USING_CACHE
	//Initialize Cache
	cacheHead = 0;
	/*for (int i = 0; i < LOG_CACHE_SIZE; i++)
	{
		(String("NA")).toCharArray(cache[i], CACHE_LINE_LENGTH);
	}*/
#endif

	LED_Init(); //If LED is available, display logic state as LED color

	//Initiate filed
	headderLine1_Names = "";
	headderLine2_Units = "";
	clearLine();
	nFields = 0;
}

void logWrite(String message)
{
	Serial.println(message);

#ifdef LOG_USING_CACHE
	//Using Cache
	if (cacheHead >= LOG_CACHE_SIZE)
	{
		//Chache overflow
		//cacheHead = 0; //Rool over (circular cache)

		return; //Do not fill cache anymore
	}

	message.toCharArray(cache[cacheHead], CACHE_LINE_LENGTH);
	cacheHead++;
#endif
}

void Log_Close()
{
	LED_Init(); //Shut down LED
	logWrite("Closing Log");
}

void Log_DumpCache()
{
#ifdef LOG_USING_CACHE
	//Begin Serial
	Serial.begin(9600);
	delay(100);

	Serial.println("Cache Dump Begin");
	Serial.println("-----------------");

	//Circular cache, the past is just about to be 'run over'
	//for (int i = cacheHead; i < LOG_CACHE_SIZE; i++)
	//	Serial.println(cache[i]);

	//Print currnent
	for (int i = 0; i < cacheHead; i++)
	{
		Serial.println(String(cache[i]));
	}

#else
	Serial.println("Not Compiled for Cache");
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////
//One line functionalities, define header, set field, clear fields
////////////////////////////////////////////////////////////////////////////////////////////////

void Log_DefineNextField (String fName, String fUnit)
{
	if (nFields >= (N_OF_LOG_FIELDS))
		logWrite("Too many loged fields, consider increasing N_OF_LOG_FIELDS");

	headderLine1_Names += padWithSpaces(fName + ",");
	headderLine2_Units += padWithSpaces(fUnit + ",");
	nFields++;
}
void Log_SetData (int fI, float data)
{
	if (fI >= (N_OF_LOG_FIELDS))
		logWrite("Too many loged fields, consider increasing N_OF_LOG_FIELDS");
	else
		lineData[fI] = data;
}
void Log_SetLoigcState(unsigned short newState)
{
	logicState = newState;
	LED_SetColor(newState); //If LED is available, display logic state as LED color
}
void Log_SetTime(unsigned long time){timems= time;}
void Log_AddNote(String note) {lineNote += String(note) + ".";}

void Log_WriteLogHeader () //Write log header
{
	//Filed
	String message1 = padWithSpaces("Time,") + headderLine1_Names + padWithSpaces("LogicState,") + " Notes";
	logWrite(message1);

	//Units
	String message2 = padWithSpaces("msec,") + headderLine2_Units + padWithSpaces(",") + "";
	logWrite(message2);
}

void Log_WriteLine ()
{
	String message = padWithSpaces(String(timems) + ",");
	for (int i=0;i<nFields;i++)
		message += padWithSpaces(String(lineData[i]) + ",");

	message += padWithSpaces(String(logicState) + ",") + lineNote;
	logWrite(message);
	clearLine();
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Tests
////////////////////////////////////////////////////////////////////////////////////////////////

//This function implements the log tester configuration
//The idea is to print a demo log
void Log_Test ()
{
	Log_Init();

	logWrite("Hello World!");

	Log_DefineNextField("Speed","m/sec");
	Log_DefineNextField("Length","m");
	Log_DefineNextField("Area","m^2");

	Log_WriteLogHeader();

	Log_SetTime(10);
	Log_SetLoigcState(10);
	Log_SetData(0,-1);   //Speed
	Log_SetData(1,2.4); //Length
	Log_SetData(2,1);  //Area
	Log_AddNote("TestData");

	Log_WriteLine();

	//Cache
	logWrite("Dumping Cache");
	Log_DumpCache();
	Log_Close();
}

void LED_Test() 
{
    LED_Init();
    for (int i = 0; i < 8; i++) 
	{
		LED_SetColor(i);
        delay(1000);
    }
}
