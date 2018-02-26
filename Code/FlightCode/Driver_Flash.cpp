#include "Driver_Flash.h"

#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV
#define FLASH_SS       SS1
#define FLASH_SPI_PORT SPI1
Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);
Adafruit_W25Q16BV_FatFs fatfs(flash);
#define FILE_NAME      "data.txt"

// IMPORTANT: before running this code, run fatfs_format.ino (example in the 
// the Adafruit_SPIFlash library) to add the FAT file system. 

File flashDataFile;

void Flash_Init()
{
    Serial.begin(9600);
	if (!flash.begin(FLASH_TYPE))
	{
		Serial.println("Could not initialize flash! See Driver_Flash.cpp");
		return;
	}
	if (!fatfs.begin())
	{
		Serial.println("Could not mount FS!");
		return;
	}
	flashDataFile = fatfs.open(FILE_NAME, FILE_WRITE);

	if (!flashDataFile)
	{
		Serial.println("Could not open file to log to! See Driver_Flash.cpp");
		return;
	}
}

void Flash_Log(String message)
{
	flashDataFile.println(message);
}

void Flash_Close()
{
	flashDataFile.close();
}
