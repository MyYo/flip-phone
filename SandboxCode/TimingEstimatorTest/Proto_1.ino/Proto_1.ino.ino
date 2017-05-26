////////////////////////////////////////////////////////////////////////////////
// Don't forget to check that all pins are assigned correctly (they currently
// are not) and to set the experiment parameters (they currently are not).
// Better yet, actually look through the code (especially the early variable
// assignment parts) to make sure that things make sense before bricking your 
// device!
////////////////////////////////////////////////////////////////////////////////
//General Includes
#include "Qduino.h"
#include "Wire.h"

//IMU Includes
#include "quaternionFilters.h"
#include "MPU9250.h"

//SD Card Includes
#include <SPI.h>
#include <SD.h>

//General Parameters
qduino q;

//SD Card Parameters
File fd;
File dataFile;
uint8_t index = 0;  //Buffer index?
const int switch_power_pin = 8;
const int switch_read_pin = 9;
const uint8_t BUFFER_SIZE = 20;
char fileName[] = "data.txt"; // SD library only supports up to 8.3 names
char buff[BUFFER_SIZE+2] = "";  // Added two to allow a 2 char peek for EOF state
const uint8_t chipSelect = 5; //Unsure what number represents
const uint8_t cardDetect = 4; //Used to detect whether SD card is present. Unsure what number means
enum states: uint8_t { NORMAL, E, EO }; //SD card states
uint8_t state = NORMAL; //Initialize the state of the SD card
bool alreadyBegan = false;  // SD.begin() misbehaves if not first call

//IMU Parameters
MPU9250 myIMU;

//Ping Parameters
int signalPinA=-1;
unsigned long pulseduration=0;
int signalPinB=-1;

//Experiment Parameters
const int maxRecordingTime = 10000;   //milliseconds
const bool logToConsole = true;
const bool logToFile = true;
const bool logToWarning = true;
const bool logToError = true;
const bool logToNormal = true;
bool gameOver = false;
bool timeExpired = false;
bool impactDetected = false;
int numWorkLoops;
float freefallGThresh = 0;            //What is the acceleration threshold beyond which we say that we are in freefall?
unsigned long fallDetectedTime = 0;   //At what time did we detect the fall?
float currentOrient[3] = {0,0,0};
float previousOrient[3] = {0,0,0};
float rotRate[3] = {0,0,0};
int timeToSimulate = 0;
int timeStep = 0;
//float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  //Variable to hold quaternion



void setup() {
  Serial.begin(9600);
  Wire.begin();

  //SD Card
  pinMode(switch_power_pin,OUTPUT);
  pinMode(switch_read_pin,INPUT);
  pinMode(cardDetect, INPUT);
  initializeCard();
  dataFile = SD.open("data.txt", FILE_WRITE);
  
  //Try to start the SD card and get it ready
  digitalWrite(switch_power_pin,HIGH);
  if (!digitalRead(cardDetect)){
    initializeCard();
  }

  //IMU
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.magCalibration);

}

void loop() {
  //Setup for file writing to SD card. 
  //Basically only do work if SD card is working.
  if(analogRead(switch_read_pin)>1000){
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if(dataFile and !gameOver){
      q.setRGB((COLORS)5);
      numWorkLoops = 0;
      myIMU.count = millis();
      normalLog("Entering Standby state");
      
      //Begin real work
      standbyState(myIMU);
     
     //Clean up after real work is done
     normalLog("Exiting Standby state, press button to play again");
     gameOver = true; 
     myIMU.delt_t = 0;
     q.ledOff();
     myIMU.count = millis();
     Serial.println(numWorkLoops);
     dataFile.println();
     dataFile.close();
    }
  }
}

//States

////////////////////////////////////////////////////////////////////////////////
// Standby state
////////////////////////////////////////////////////////////////////////////////
void standbyState(MPU9250 &myIMU)
{
  while(myIMU.delt_t<maxRecordingTime and !timeExpired and !impactDetected){
    myIMU.delt_t = millis() - myIMU.count;
    numWorkLoops+=1;
    updateIMU(myIMU);
    //updateOrientRot(myIMU);
    normalLog(stringifyIMU(myIMU));
    

    //If we detect that we are in freefall
    if(accMag(myIMU) < freefallGThresh){
      fallDetectedTime = millis();

      //Generate time-orientation table
      buildTOTable(currentOrient, fallDetectedTime, rotRate);
      
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Distance Acquisition state
////////////////////////////////////////////////////////////////////////////////
void distAcquisitionState(MPU9250 &myIMU)
{
  while(myIMU.delt_t<maxRecordingTime and !timeExpired and !impactDetected){
    myIMU.delt_t = millis() - myIMU.count;

    //Read orientation at current time point. If orientation is ground, then measure.
    measureDistance(signalPinA);

    //If we have measured some predetermined number of distances, transition
    //to Impact Forecaster state
      
    
  }
}

//To be implemented working methods

////////////////////////////////////////////////////////////////////////////////
// Build time-orientation table that has entries: Time, Orientation
// using equations from FlipPhone Software Design slide 8, Generate Time-Predicted Orientation Table
////////////////////////////////////////////////////////////////////////////////
void buildTOTable(float initOrient[], float initTime, float initRot[])
{

}

//Stable working methods

////////////////////////////////////////////////////////////////////////////////
// Turn on the ping, wait, read the ping, determine time until response
////////////////////////////////////////////////////////////////////////////////
int measureDistance(int &signal)
{
 // set pin as output so we can send a pulse
 pinMode(signal, OUTPUT);
// set output to LOW
 digitalWrite(signal, LOW);
 delayMicroseconds(5);
 
 // now send the 5uS pulse out to activate Ping)))
 digitalWrite(signal, HIGH);
 delayMicroseconds(5);
 digitalWrite(signal, LOW);
 
 // now we need to change the digital pin
 // to input to read the incoming pulse
 pinMode(signal, INPUT);
 
 // finally, measure the length of the incoming pulse
 pulseduration=pulseIn(signal, HIGH);
  // divide the pulse length by half
 pulseduration=pulseduration/2; 
 
 // now convert to centimetres. We're metric here people...
 int distance = int(pulseduration/29);
 return distance;
}

////////////////////////////////////////////////////////////////////////////////
// Update IMU object to get latest IMU values
////////////////////////////////////////////////////////////////////////////////
void updateIMU(MPU9250 &myIMU)
{
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01){
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
            myIMU.getAres();
    
            // Now we'll calculate the accleration value into actual g's
            // This depends on scale being set
            myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
            myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
            myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];
    
            myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
            myIMU.getGres();
    
            // Calculate the gyro value into actual degrees per second
            // This depends on scale being set
            myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
            myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
            myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;
    
            myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
            myIMU.getMres();
            // User environmental x-axis correction in milliGauss, should be
            // automatically calculated
            myIMU.magbias[0] = +470.;
            // User environmental x-axis correction in milliGauss TODO axis??
            myIMU.magbias[1] = +120.;
            // User environmental x-axis correction in milliGauss
            myIMU.magbias[2] = +125.;
    
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental
            // corrections
            // Get actual magnetometer value, this depends on scale being set
            myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                   myIMU.magbias[0];
            myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                   myIMU.magbias[1];
            myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                   myIMU.magbias[2];
          } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  
          // Must be called before updating quaternions!
          myIMU.updateTime();
  
          // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
          // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
          // (+ up) of accelerometer and gyro! We have to make some allowance for this
          // orientationmismatch in feeding the output to the quaternion filter. For the
          // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
          // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
          // modified to allow any convenient orientation convention. This is ok by
          // aircraft orientation standards! Pass gyro rate as rad/s
          // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
          MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                            myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                            myIMU.mx, myIMU.mz, myIMU.deltat);
          
}


////////////////////////////////////////////////////////////////////////////////
// Stringify updated IMU values
////////////////////////////////////////////////////////////////////////////////
String stringifyIMU(MPU9250 &myIMU)
{
  String currenttime = String(myIMU.delt_t);
  String ax = String(myIMU.ax);
  String ay = String(myIMU.ay);
  String az = String(myIMU.az);
  String amag = String(accMag(myIMU));
  String gx = String(myIMU.gx);
  String gy = String(myIMU.gy);
  String gz = String(myIMU.gz);
  String mx = String(myIMU.mx);
  String my = String(myIMU.my);
  String mz = String(myIMU.mz);
  String output = currenttime + "," + ax + "," + ay + "," + az + "," + amag + "," + gx + "," + gy + "," + gz + "," + mx + "," + my + "," + mz;
  return output;
}

////////////////////////////////////////////////////////////////////////////////
// Arrayify updated IMU values
////////////////////////////////////////////////////////////////////////////////
void arrayifyIMU(MPU9250 &myIMU, float dataString[])
{
  dataString[0] = myIMU.delt_t;
  dataString[1] = myIMU.ax;
  dataString[2] = myIMU.ay;
  dataString[3] = myIMU.az;
  dataString[4] = accMag(myIMU);
  dataString[5] = myIMU.gx;
  dataString[6] = myIMU.gy;
  dataString[7] = myIMU.gz;
  dataString[8] = myIMU.mx;
  dataString[9] = myIMU.my;
  dataString[10] = myIMU.mz;
}

////////////////////////////////////////////////////////////////////////////////
// Get acceleration magnitude
////////////////////////////////////////////////////////////////////////////////
double accMag(MPU9250 &myIMU)
{
  return sqrt(pow(myIMU.ax,2)+pow(myIMU.ay,2)+pow(myIMU.az,2));
}

////////////////////////////////////////////////////////////////////////////////
// Update orientation and rotation rate local variables
////////////////////////////////////////////////////////////////////////////////
void updateOrientRot(MPU9250 &myIMU)
{
  rotRate[0] = currentOrient[0] - previousOrient[0];
  rotRate[1] = currentOrient[1] - previousOrient[1];
  rotRate[2] = currentOrient[2] - previousOrient[2];

  previousOrient[0] = currentOrient[0];
  previousOrient[1] = currentOrient[1];
  previousOrient[2] = currentOrient[2];
  
  currentOrient[0] = myIMU.gx;
  currentOrient[1] = myIMU.gy;
  currentOrient[2] = myIMU.gz;
}


//Logging Methods

////////////////////////////////////////////////////////////////////////////////
// Log normal message
////////////////////////////////////////////////////////////////////////////////
void normalLog(String message)
{
  if (logToNormal){
    if (logToConsole){
      Serial.println("Log: " + message);
    }
    if (logToFile){
      dataFile.println("Log: " + message);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Log warning message
////////////////////////////////////////////////////////////////////////////////
void warningLog(String message)
{
  if (logToWarning){
    if (logToConsole){
      Serial.println("Warning Log: " + message);
    }
    if (logToFile){
      dataFile.println("Warning Log: " + message);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Log error message
////////////////////////////////////////////////////////////////////////////////
void errorLog(String message)
{
  if (logToError){
    if (logToConsole){
      Serial.println("Error Log: " + message);
    }
    if (logToFile){
      dataFile.println("Error Log: " + message);
    }
  }
}

//Setup Methods

////////////////////////////////////////////////////////////////////////////////
// Do everything from detecting card through opening the demo file
////////////////////////////////////////////////////////////////////////////////
void initializeCard(void)
{
  Serial.print(F("Initializing SD card..."));

  // Is there even a card?
  if (!digitalRead(cardDetect))
  {
    Serial.println(F("No card detected. Waiting for card."));
    while (!digitalRead(cardDetect));
    delay(250); // 'Debounce insertion'
  }

  // Card seems to exist.  begin() returns failure
  // even if it worked if it's not the first call.
  if (!SD.begin(chipSelect) && !alreadyBegan)  // begin uses half-speed...
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

  Serial.println(F("Enter text to be written to file. 'EOF' will terminate writing."));
}

////////////////////////////////////////////////////////////////////////////////
// This function is called after the EOF command is received. It writes the
// remaining unwritten data to the µSD card, and prints out the full contents
// of the log file.
////////////////////////////////////////////////////////////////////////////////
void eof(void)
{
  index -= 3; // Remove EOF from the end
  flushBuffer();

  // Re-open the file for reading:
  fd = SD.open(fileName);
  if (fd)
  {
    Serial.println("");
    Serial.print(fileName);
    Serial.println(":");

    while (fd.available())
    {
      Serial.write(fd.read());
    }
  }
  else
  {
    Serial.print("Error opening ");
    Serial.println(fileName);
  }
  fd.close();
}


////////////////////////////////////////////////////////////////////////////////
// Write the buffer to the log file. If we are possibly in the EOF state, verify
// that to make sure the command isn't written to the file.
////////////////////////////////////////////////////////////////////////////////
void flushBuffer(void)
{
  fd = SD.open(fileName, FILE_WRITE);
  if (fd) {
    switch (state)  // If a flush occurs in the 'E' or the 'EO' state, read more to detect EOF
    {
    case NORMAL:
      break;
    case E:
      readByte();
      readByte();
      break;
    case EO:
      readByte();
      break;
    }
    fd.write(buff, index);
    fd.flush();
    index = 0;
    fd.close();
  }
}



////////////////////////////////////////////////////////////////////////////////
// Reads a byte from the serial connection. This also maintains the state to
// capture the EOF command.
////////////////////////////////////////////////////////////////////////////////
void readByte(void)
{
  byte byteRead = Serial.read();
  Serial.write(byteRead); // Echo
  buff[index++] = byteRead;

  // Must be 'EOF' to not get confused with words such as 'takeoff' or 'writeoff'
  if (byteRead == 'E' && state == NORMAL)
  {
    state = E;
  }
  else if (byteRead == 'O' && state == E)
  {
    state = EO;
  }
  else if (byteRead == 'F' && state == EO)
  {
    eof();
    state = NORMAL;
  }
}

