#include "Driver_Motor.h"
#include "Arduino.h"
#include "HardwareConfiguration.h"

void   Motor_Init ()
{
    //Init pins as output
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_INA, OUTPUT);
    pinMode(PIN_MOTOR_INB, OUTPUT);

    //Lock the motor so it does not spin
    Motor_Break();
    
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_StartForward ()
{
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, HIGH);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
  
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_StartBackward ()
{
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, HIGH);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

//Make sure you call this function once, otherwise the motor will start-break cycle.
void   Motor_Break ()
{
  digitalWrite(PIN_MOTOR_PWM, LOW); //Before changing direction turn off the motor
  digitalWrite(PIN_MOTOR_INA, LOW);
  digitalWrite(PIN_MOTOR_INB, LOW);
  digitalWrite(PIN_MOTOR_PWM, HIGH); //Turn motor back on
}

float Motor_MeasureMotorDriverInputVoltage()
{
	const float vdd = 3.3; //[V] - reference voltage 
	const float divider = 2 * 0.989;//Voltage devider factor to convert from measured voltage to motor driver input voltage
	float analogValue = ((float)analogRead(PIN_CAPACITOR_VOLTAGE)) / 1024.0* vdd;

	return analogValue * divider;
}

void Motor_Test () //Tester function
{
  //Start and stop motor
  Motor_Init();
  delay(500);
  Motor_StartForward();
  delay(300);
  Motor_Break();
  delay(500);
  Motor_StartBackward();
  delay(500);
}

void Motor_TestMeasureCapacitorDriverInputVoltage() //Tester function
{
	//Continuesly measure voltage and output it
	while (true)
	{
		Serial.print("Capacitor Voltage: ");
		Serial.print(Motor_MeasureMotorDriverInputVoltage());
		Serial.println(" [V]");
		delay(500);
	}
}
