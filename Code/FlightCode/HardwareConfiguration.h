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

//Select whether to log data through the com connection or SD card
#define LOG_TO_COM
//#define LOG_TO_SD
//#define LOG_TO_FLASH

#endif
