#include "HardwareConfiguration.h"
#include "Driver_SDCard.h"

#include "Wire.h"


#include <SPI.h>
#include <SD.h>

char fileName[] = "data.txt"; // SD library only supports up to 8.3 names
File dataFile;
bool alreadyBegan = false; // SD.begin() misbehaves if not first call

//Initializes Hardware
void initializeCard(void);
void SD_Init ()
{
	Serial.begin(9600);
	Wire.begin();

	//Configure Hardware
	pinMode(PIN_SD_CARD_DETECT, INPUT); //Used to detect whether SD card is present

	initializeCard();
	dataFile = SD.open("data.txt", FILE_WRITE);
  
	//Try to start the SD card and get it ready
	if (!digitalRead(PIN_SD_CARD_DETECT))
	{
		initializeCard();
	}
}

// Do everything from detecting card through opening the demo file
void initializeCard(void)
{
  Serial.print(F("Initializing SD card..."));

  // Is there even a card?
  if (!digitalRead(PIN_SD_CARD_DETECT))
  {
    Serial.println(F("No card detected. Waiting for card."));
    while (!digitalRead(PIN_SD_CARD_DETECT));
    delay(250); // 'Debounce insertion'
  }

  // Card seems to exist.  begin() returns failure
  // even if it worked if it's not the first call.
  if (!SD.begin(PIN_SD_CHIP_SELECT) && !alreadyBegan)  // begin uses half-speed...
  {
    Serial.println(F("Initialization failed!"));
    initializeCard(); // Possible infinite retry loop is as valid as anything
  }
  else
  {
    alreadyBegan = true;
  }
  Serial.println(F("Initialization done."));

  Serial.print(fileName);
  if (SD.exists(fileName))
  {
    Serial.println(F(" exists."));
  }
  else
  {
    Serial.println(F(" doesn't exist. Creating."));
  }

  Serial.print("Opening file: ");
  Serial.println(fileName);
}
	
void   SD_Log (String Message)
{
	dataFile.println(Message);
}

void   SD_Close()
{
	dataFile.println();
	dataFile.close();
}
