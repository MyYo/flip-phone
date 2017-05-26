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

//Ping Parameters
int signalPinA=-1;
unsigned long pulseduration=0;
int signalPinB=-1;

//Experiment Parameters
const int maxRecordingTime = 10000;   //milliseconds
const int numGroundsToRecord = 3;     //How many times do you want to see the ground before predicting impact time?
float freefallGThresh = 0;            //What is the acceleration threshold beyond which we say that we are in freefall?
float groundAngleMinA = 0;            //At what minimum angle do we start collecting distance data for sensor A?
float groundAngleMaxA = 0;            //At what maximum angle do we stop collecting distance data for sensor A?
float groundAngleMinB = 0;            //At what minimum angle do we start collecting distance data for sensor B?
float groundAngleMaxB = 0;            //At what maximum angle do we stop collecting distance data for sensor B?
const int numGroundTempsToRecord = 10;          //How many distance measurements should we collect for each specific ground?
float landedGThresh = 0;              //What is the acceleration threshold after which we have impacted?

//Experiment Variables
unsigned long fallDetectedTime = 0;   //At what time did we detect the fall?
int groundsCounter = 0;               //How many grounds have we seen thus far?
unsigned long groundTimes[numGroundsToRecord];  //Array containing the time at which a ground was seen
long groundDistances[numGroundsToRecord];       //Array containing the distances measured when a ground was seen
unsigned long predictedImpactTime = 0;          //At what time do we predict that we will impact the ground?
int groundTempCounter = 0;                      //How many distance measurements have we seen already for this specific ground?
unsigned long groundTimesTemp[numGroundTempsToRecord];    //A buffer to hold all the time measurements for this specific ground
long groundDistancesTemp[numGroundTempsToRecord];         //A buffer to hold all the distance measurements for this specific ground
bool impactTimeEstimated = false;
long actualImpactTime = 0;            //Actual impact time
long estImpactTime = 0;               //Estimated impact time
//States:
//0 - Off
//1 - Setup complete
//2 - Start looping
//3 - Fall detected
//4 - Falling
//5 - Ground detected
//6 - Impact occurred
uint8_t physicsState = 0;

//IMU Parameters
MPU9250 myIMU;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  //SD Card
  pinMode(switch_power_pin,OUTPUT);
  pinMode(switch_read_pin,INPUT);
  pinMode(cardDetect, INPUT);
  initializeCard();
  File dataFile = SD.open("data.txt", FILE_WRITE);
  
  //Ping Sensors
  pinMode(signalPinA,OUTPUT);
  pinMode(signalPinB,OUTPUT);

  //IMU
  myIMU.initMPU9250();
  myIMU.initAK8963(myIMU.magCalibration);

  //Experiment
  physicsState = 1;
}

void loop() {
  
  physicsState = 2;
  
  //Try to start the SD card and get it ready
  digitalWrite(switch_power_pin,HIGH);
  if (!digitalRead(cardDetect)){
    initializeCard();
  }
  
  while(true){

    //Setup for file writing to SD card
    myIMU.count = millis();
    if(analogRead(switch_read_pin)>1000){
      File dataFile = SD.open("data.txt", FILE_WRITE);
      if(dataFile){
        q.setRGB((COLORS)5);
        int j = 0;

        //While the experiment time limit is not reached
        while(myIMU.delt_t<maxRecordingTime){
          myIMU.delt_t = millis() - myIMU.count;
          j+=1;

          //This function not only gets the current IMU data,
          //but also grabs the IMU data and prepares it for retrieval
          float dataString[11];
          imuReadVals(myIMU, dataString);
          dataFile.println(imuReadValsStr(dataString));

          //If we detect that we are in freefall and we were not
          //previously in freefall, then set our state to freefall
          //and save the time in which we detected freefall for later use.
          if((myIMU.ax < freefallGThresh) & physicsState < 3){
            physicsState = 3;
            fallDetectedTime = millis();
            physicsState = 4;
          }

          //If we impacted after being in freefall, we should notify the user
          //and calculate difference between estimated and actual, if estimation completed.
          //Otherwise, continue to estimate impact time.
          if((physicsState == 4 | physicsState == 5) & (myIMU.ax > landedGThresh)){
            if(actualImpactTime == 0) {
              actualImpactTime == millis();
              Serial.println("Impact occurred!");
            }
            if(impactTimeEstimated) {
              Serial.println("Impact Time Estimated: " + String(estImpactTime));
              Serial.println("Actual Impact Time: " + String(actualImpactTime));
              Serial.println("Delta Time: " + String(actualImpactTime - estImpactTime));
            }
            else {
              Serial.println("Impact occurred before impactTimeEstimated");
            }
          }
          else {
            //If we are currently in freefall or have already been facing the ground, and sensor A is facing the ground,
            //and we have not exceeded the number of measurements for this specific ground, take another measurement.
            if( (physicsState == 4 | physicsState == 5) & myIMU.gx > groundAngleMinA & myIMU.gx < groundAngleMaxA & groundsCounter <= numGroundsToRecord) {
              physicsState = 5;
              if(groundTempCounter < numGroundTempsToRecord){
                groundTimesTemp[groundTempCounter] = millis();
                groundDistancesTemp[groundTempCounter] = measureDistance(signalPinA);
                groundTempCounter+=1;  
              }
              
            }
            //If we are currently in freefall or have already been facing the ground, and sensor B is facing the ground,
            //and we have not exceeded the number of measurements for this specific ground, take another measurement.
            else if( (physicsState == 4 | physicsState == 5) & myIMU.gx > groundAngleMinB & myIMU.gx < groundAngleMaxB & groundsCounter <= numGroundsToRecord) {
              physicsState = 5;
              if(groundTempCounter < numGroundTempsToRecord){
                groundTimesTemp[groundTempCounter] = millis();
                groundDistancesTemp[groundTempCounter] = measureDistance(signalPinB);
                groundTempCounter+=1;  
              }
              
            }
            //If we were previously facing the ground, but are no longer facing the ground, and we have not met
            //our limit on number of grounds to record, we need to change the state back to freefall. We also
            //have to reset the ground counter for the specific ground back to zero, as well as the respective buffers.
            //Finally, we must also find the minimum from the buffers and choose that as the actual ground distance.
            else if (physicsState == 5 & groundsCounter <= numGroundsToRecord){
              physicsState = 4;
              groundTempCounter = 0;
              int minIndex = indexOfMinNonZero(groundDistancesTemp);
              groundTimes[groundsCounter] = groundTimesTemp[minIndex] - fallDetectedTime;
              groundDistances[groundsCounter] = groundDistancesTemp[minIndex];
              groundsCounter+=1;
            }
            //If we have collected enough ground data, we should calculate the actual ground distance for the final
            //ground then calculate the time to impact from all the data we have collected
            else if (groundsCounter > numGroundsToRecord & !impactTimeEstimated) {
              estImpactTime = estimatedImpactTime(fallDetectedTime, groundTimes, groundDistances);
              impactTimeEstimated = true;
              
            }
          }
          
          
          
       }

       //Clean up after the experiment time limit is up
       myIMU.delt_t = 0;
       q.ledOff();
       myIMU.count = millis();
       Serial.println(j);
       dataFile.println();
       dataFile.close();
      }
    }  
  } 
}

////////////////////////////////////////////////////////////////////////////////
// Just get me the IMU values in an array
////////////////////////////////////////////////////////////////////////////////
void imuReadVals(MPU9250 &myIMU, float dataString[])
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
          String currenttime = String(myIMU.delt_t);
          String ax = String(myIMU.ax);
          String ay = String(myIMU.ay);
          String az = String(myIMU.az);
          String amag = String(sqrt(pow(myIMU.ax,2)+pow(myIMU.ay,2)+pow(myIMU.az,2)));
          String gx = String(myIMU.gx);
          String gy = String(myIMU.gy);
          String gz = String(myIMU.gz);
          String mx = String(myIMU.mx);
          String my = String(myIMU.my);
          String mz = String(myIMU.mz);
          //dataString = {myIMU.delt_t,myIMU.ax,myIMU.ay,myIMU.az,sqrt(pow(myIMU.ax,2)+pow(myIMU.ay,2)+pow(myIMU.az,2)),myIMU.gx,myIMU.gy,myIMU.gz,myIMU.mx,myIMU.my,myIMU.mz}
          dataString[0] = myIMU.delt_t;
          dataString[1] = myIMU.ax;
          dataString[2] = myIMU.ay;
          dataString[3] = myIMU.az;
          dataString[4] = sqrt(pow(myIMU.ax,2)+pow(myIMU.ay,2)+pow(myIMU.az,2));
          dataString[5] = myIMU.gx;
          dataString[6] = myIMU.gy;
          dataString[7] = myIMU.gz;
          dataString[8] = myIMU.mx;
          dataString[9] = myIMU.my;
          dataString[10] = myIMU.mz;
          imuReadValsStr(dataString);
}

////////////////////////////////////////////////////////////////////////////////
// Average all values that are non zero in an array
////////////////////////////////////////////////////////////////////////////////
double averageNonZero(long arr[])
{
  long sum;
  int ctr = 0;
  for(int i=0; i<(sizeof(arr)/sizeof(arr[0])); i++){
    if(arr[i] != 0){
      sum+= arr[i];
      ctr+=1;
    }
  return sum / ((double) ctr);
     
  }
}


////////////////////////////////////////////////////////////////////////////////
// Convert IMU array to string
////////////////////////////////////////////////////////////////////////////////
String imuReadValsStr(float arr[])
{
  String dataString = String(String(arr[0])+" "+String(arr[1])+" "+String(arr[2])+" "+String(arr[3])+" "+String(arr[4])+" "+String(arr[5])+" "+String(arr[6])+" "+String(arr[7])+" "+String(arr[8])+" "+String(arr[9])+" "+String(arr[10]));
  Serial.println(dataString);
  return dataString;
}

////////////////////////////////////////////////////////////////////////////////
// Get current time in millis
////////////////////////////////////////////////////////////////////////////////
double getCurrentTimeMillis()
{
  return millis();
}

////////////////////////////////////////////////////////////////////////////////
// Get current acceleration
////////////////////////////////////////////////////////////////////////////////
void getAccel(float arr[], float output[])
{
  output[0] = arr[1];
  output[1] = arr[2];
  output[2] = arr[3];
  output[3] = arr[4];
  
}

////////////////////////////////////////////////////////////////////////////////
// Get current yaw/pitch/roll
////////////////////////////////////////////////////////////////////////////////
void getGyro(float arr[], float output[])
{
  output[0] = arr[5];
  output[1] = arr[6];
  output[2] = arr[7];
  
}

////////////////////////////////////////////////////////////////////////////////
// Get current rotation rate
////////////////////////////////////////////////////////////////////////////////
void getGyro(float arrOld[], float arrNew[], float output[])
{
  output[0] = arrOld[0]-arrNew[0];
  output[1] = arrOld[1]-arrNew[1];
  output[2] = arrOld[2]-arrNew[2];
}

////////////////////////////////////////////////////////////////////////////////
// Get current magnetic field
////////////////////////////////////////////////////////////////////////////////
void getMag(float arr[], float output[])
{
  output[0] = arr[8];
  output[1] = arr[9];
  output[2] = arr[10];
}

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
// Build distance-angle table that has entries: Time, Orientation, Distance To Ground
// using equations from FlipPhone Software Design slide 8, Generate Time-Predicted Orientation Table
////////////////////////////////////////////////////////////////////////////////
void buildDATable(float initOrient[], float initTime, float initRot[], int timeToSimulate, int timeStep)
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Find the estimated impact time based on time when fall was detected, the
// multiple ground distance measurements and the associated times. This
// calculation can be found in FlipPhone Software Design slides 9 and 10, 
// Impact Forecaster
////////////////////////////////////////////////////////////////////////////////
long estimatedImpactTime(long fallDetectedTime, unsigned long groundTimes[], long groundDistances[]) {
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Get initial velocity: First use two distance and paired time data and
// kinematic equation d = v_i * t + 1/2 * a * t^2 to solve for v_i, the velocity
// at the earlier of the two distance measurements. Then, use that velocity as
// v_f and solve for v_i, velocity at fall start, using v_f = v_i + a * t, where
// t = time since fall start. 
////////////////////////////////////////////////////////////////////////////////
long velocityAtFall(long fallDetectedTime, unsigned long groundTimes[], long groundDistances[]) {
  return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Find the element of minimum non-zero value in an array
////////////////////////////////////////////////////////////////////////////////
int indexOfMinNonZero(long arr[])
{
  long minimumVal=0;
  int minimumIndex;
  for(int i=0; i<(sizeof(arr)/sizeof(arr[0])); i++){
    if(arr[i] != 0){
      if(minimumVal == 0){
        minimumVal = arr[i];
        minimumIndex = i;
      }
      else if(arr[i] < minimumVal) {
        minimumVal = arr[i];
        minimumIndex = i;        
      }
    }
  return minimumIndex;
     
  }
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
