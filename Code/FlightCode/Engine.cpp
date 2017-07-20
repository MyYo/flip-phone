#include "Engine.h"
#include "logger.h"
#include "Arduino.h"
#include "HardwareConfiguration.h"

void   Eng_Init ()
{
    //Init pins as output
    Log_AddNote("Initializing Engine");
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_INA, OUTPUT);
    pinMode(PIN_MOTOR_INB, OUTPUT);

    //Lock the engine so it does not spin
    Eng_Break();
    
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Eng_StartForward ()
{
  Log_AddNote("Set Engine FW");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, HIGH);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
  
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Eng_StartBackward ()
{
  Log_AddNote("Set Engine BW");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, HIGH);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Eng_Break ()
{
  Log_AddNote("Breaking Engine");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

void   Eng_Test () //Tester function
{
	//Start and stop engine
  Eng_Init();
  delay(500);
  Eng_StartForward();
  delay(500);
  Eng_Break();
  delay(500);
  Eng_StartBackward();
  delay(500);
}
