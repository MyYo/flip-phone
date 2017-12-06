// flash_benchmark.ino 
// Runs a simple flash benchmark on the Feather M0 Express

#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>
#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV
#define FLASH_SS       SS1
#define FLASH_SPI_PORT SPI1
Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);
Adafruit_W25Q16BV_FatFs fatfs(flash);
#define FILE_NAME      "data.csv"
#define CYCLES         100
#define LOGS_PER_FALL  20
#define LOG_LENGTH     40
#define PRINT_EVERY    10

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Serial connected.");
    if (!flash.begin(FLASH_TYPE)) {
        Serial.println("ERROR: could not initialize flash.");
        while(1);
    }
    if (!fatfs.begin()) {
        Serial.println("ERROR: could not mount FS.");
        while(1);
    }
  
    String payload = "";
    for (int k = 0; k < LOG_LENGTH; k++) {
        payload += 'x';
    }
    
    float start_time = millis();
    float total_open_time = 0;
    float total_write_time = 0;
    
    for (int i = 0; i < CYCLES; i++) {
        if (i % PRINT_EVERY == 0) {
            Serial.print(i);
            Serial.println(" cycles completed.");
        }
        float open_start = millis();
        File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
        float open_end = millis();
        total_open_time += open_end - open_start;
        if (dataFile) {
            float write_start = millis();
            for (int j = 0; j < LOGS_PER_FALL; j++) {
                dataFile.println(payload);
            }
            float write_end = millis();
            total_write_time += (write_end - write_start);
        } else {
            Serial.println("Failed to open data file for writing!");
            while(1);
        }
        dataFile.close();
    }
    float end_time = millis();
    float total_time = end_time - start_time;
    
    Serial.print("Wrote ");
    Serial.print(CYCLES);
    Serial.print(" cycles with ");
    Serial.print(LOGS_PER_FALL);
    Serial.print(" logs per fall in ");
    Serial.print(total_time / 1000.0);
    Serial.println(" seconds.");

    Serial.print("Average time per cycle: ");
    Serial.print(total_time / CYCLES);
    Serial.println(" ms");
    
    Serial.print("Average time per file open: ");
    Serial.print(total_open_time / (CYCLES));
    Serial.println(" ms");
    
    Serial.print("Average time per file write: ");
    Serial.print(total_write_time / (CYCLES * LOGS_PER_FALL));
    Serial.println(" ms");

}

void loop() {}
