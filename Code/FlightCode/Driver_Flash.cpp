#include "Driver_Flash.h"

#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV
#define FLASH_SS       SS1
#define FLASH_SPI_PORT SPI1
#define FILE_NAME      "data.txt"

File dataFile;
Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);
Adafruit_W25Q16BV_FatFs fatfs(flash);

void Flash_Init()
{
    if (!flash.begin(FLASH_TYPE)) 
    {
        Serial.println("Could not initialize flash!");
        return;
    }
    if (!fatfs.begin()) 
    {
        Serial.println("Could not mount FS!");
        return;
    }
    dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
    if (!dataFile) 
    {
        Serial.println("Could not open file to log to!");
    }
}

void Flash_Log(String message)
{
	dataFile.println(message);
}

void Flash_Close()
{
	dataFile.close();
}
