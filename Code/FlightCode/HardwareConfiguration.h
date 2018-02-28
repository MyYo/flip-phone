#ifndef HARDWARE_CONFIGURATION_H
#define HARDWARE_CONFIGURATION_H
//This file defines the hardware configuration of the system

//Pin table (Re-define depending on actual hardware configuration)
#define PIN_SD_CARD_DETECT   9	 //SDCard Port CD
#define PIN_SD_CHIP_SELECT   8	 //SDCard Port CS 
#define PIN_PING_DOWNFACING  A0  //Ping Port SIG
#define PIN_PING_UPFACING    A1  //Ping Port SIG
#define PIN_I2C_SCL		     A5	 //Used by IMU
#define PIN_I2C_SDA		     A4	 //Used by IMU
#define PIN_MOTOR_INA  11  //Motor
#define PIN_MOTOR_PWM  10  //Motor
#define PIN_MOTOR_INB   9  //Motor

//Safety and debug
#define PIN_SAFETY_PLUG_SOURCE 5 
#define PIN_SAFETY_PLUG_TERMINAL 6
#define PIN_CAPACITOR_VOLTAGE A2
#define PIN_LED_VOLTAGE_OK_INDICATOR 13 //When LED is on voltage is OK

//Select whether to log data through the com connection or SD card
#define LOG_TO_COM
//#define LOG_TO_FLASH
#define LOG_USING_CACHE //When defined, log will be cashed and written upon closure. Comment out to log on-the-fly
#endif
