#ifndef _LOGGER_H_
#define _LOGGER_H_

#include "Arduino.h"

void Log_Init ();
void Log_Close ();
void Log_DefineNextField (String fName, String fUnit); //Define header
void Log_SetData (int fI, float data);
void Log_SetLoigcState(unsigned short newState);
void Log_SetTime(unsigned long time);
void Log_AddNote(String note); // Can be used for warnings or errors, or just additional information

void Log_WriteLogHeader (); //Write log header
void Log_WriteLine (); //Write data to log, no note

void Log_Test (); //Tests main functions of loger

#endif
