#include "Engine.h"
#include "logger.h"
#include "Arduino.h"

void   Eng_Init ()
{
	//TBD: Do magic
}
void   Eng_SetDirection (int dir) //1 - froward, -1 backward
{
	if (dir==1)
		Log_AddNote("Set Engine FW");
	else
		Log_AddNote("Set Engine BW");

	//TBD: Do magic
}
void   Eng_Start ()
{
	//TBD: Do magic
}
void   Eng_Break ()
{
	//TBD: Do magic
}

void   Eng_Test () //Tester function
{
	//Start and stop engine
	//TBD
}
