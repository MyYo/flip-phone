#include "Motor.h"
#include "logger.h"
#include "Arduino.h"
#include "HardwareConfiguration.h"

void   Motor_Init ()
{
    //Init pins as output
    Log_AddNote("Initializing Motor");
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_INA, OUTPUT);
    pinMode(PIN_MOTOR_INB, OUTPUT);

    //Lock the motor so it does not spin
    Motor_Break();
    
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_StartForward ()
{
  Log_AddNote("Set Motor FW");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, HIGH);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
  
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_StartBackward ()
{
  Log_AddNote("Set Motor BW");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, HIGH);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_Break ()
{
  Log_AddNote("Breaking Motor");
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

void   Motor_Test () //Tester function
{
	//Start and stop motor
  Motor_Init();
  delay(500);
  Motor_StartForward();
  delay(500);
  Motor_Break();
  delay(500);
  Motor_StartBackward();
  delay(500);
}
