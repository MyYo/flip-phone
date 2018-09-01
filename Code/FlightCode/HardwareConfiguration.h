#ifndef HARDWARE_CONFIGURATION_H
#define HARDWARE_CONFIGURATION_H
//This file defines the hardware configuration of the system

//Pin table (Re-define depending on actual hardware configuration)
//#define PIN_SD_CARD_DETECT   9	 //SDCard Port CD
//#define PIN_SD_CHIP_SELECT   8	 //SDCard Port CS 
#define PIN_PING_DOWNFACING  A1  //Ping Port SIG
#define PIN_PING_UPFACING    A2  //Ping Port SIG
#define PIN_I2C_SCL		     A5	 //Used by IMU
#define PIN_I2C_SDA		     A4	 //Used by IMU
#define PIN_MOTOR_INA  11  //Motor
#define PIN_MOTOR_PWM  10  //Motor
#define PIN_MOTOR_INB   9  //Motor

//Safety and debug
#define PIN_SAFETY_PLUG_SOURCE 5 
#define PIN_SAFETY_PLUG_TERMINAL 6
#define PIN_CAPACITOR_VOLTAGE A5		//Power meeter pin. Measuring voltage on capacitor
#define PIN_LED_VOLTAGE_OK_INDICATOR 13 //When LED is on voltage is OK

//Motor - Mechanical
#define WHEEL_CASE_RATIO_R 18.465         //[Unitless] Set to:
										  //	- Only case: 18.465
										  //    - Case with phone: 
#define MOTOR_PARAMETER_E  266.9          //[rad/secV]
#define MOTOR_PARAMETER_TC (323.5/1000.0) //[sec]

//Logging
#define LOG_USING_CACHE //When defined, log will also be cashed and written upon closure.
#endif
