#ifndef HARDWARE_CONFIGURATION_H
#define HARDWARE_CONFIGURATION_H
//This file defines the hardware configuration of the system

//Pin table (Re-define depending on actual hardware configuration)
#define PIN_SD_SWITCH_POWER		8
#define PIN_SD_SWITCH_READ		9
#define PIN_SD_CARD_DETECT		4
#define PIN_SD_CHIP_SELECT		5
#define PIN_PING_DOWNFACING		1
#define PIN_PING_UPFACING		1
#define PIN_I2C_SCL				A5  //Used by IMU
#define PIN_I2C_SDA				A4  //Used by IMU


//Select whether to log data through the com connection or SD card
#define LOG_TO_COM
//#define LOG_TO_SD


#endif