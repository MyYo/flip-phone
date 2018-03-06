//This is higher level driver wrapper of Driver_IMU_MPU9250
#include "HardwareConfiguration.h"
#include "Driver_SafetyPin.h"
#include "Arduino.h"

void SafetyPin_Init()
{
	//Initiate Safety Plug
	pinMode(PIN_SAFETY_PLUG_SOURCE, OUTPUT);
	pinMode(PIN_SAFETY_PLUG_TERMINAL, INPUT_PULLUP);
	digitalWrite(PIN_SAFETY_PLUG_SOURCE, LOW);
}

bool SafetyPin_IsConnected() //Returns true if safety pin connected
{
	return !digitalRead(PIN_SAFETY_PLUG_TERMINAL);
}
