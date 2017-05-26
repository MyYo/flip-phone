#ifndef _ENGINE_H_
#define _ENGINE_H_

void   Eng_Init ();   //Initializes Hardware
void   Eng_SetDirection (int dir); //1 - froward, -1 backward
void   Eng_Start ();
void   Eng_Break ();

void   Eng_Test (); //Tester function

#endif