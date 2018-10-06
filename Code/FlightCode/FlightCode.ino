////////////////////////////////////////////////////////////////////////////////
//Make sure you downolad Arduino libraries used here:
// SPI Flash: https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-spi-flash (Not in use)
// Neo Pixel: https://github.com/adafruit/Adafruit_NeoPixel
// In the above menue select Sketch -> Include Library -> Add .Zip Library and select downloded zips
// Compile for Adafruit Feather M0 Express

//General Includes
#include "HardwareConfiguration.h"
#include "Logger.h"

//#define IS_RUN_OPERATIONAL //Comment out if you would like to run tests only

void setup()
{
  //All setup is done in the logic code
}

#ifdef IS_RUN_OPERATIONAL
#include "Logic.h"

void loop()
{
  //Operational mode
  delay(2000); //delay before starting
  RunLogic();
}
#else

#include "Driver_IMU.h"
#include "Driver_Distance.h"
#include "Driver_Motor.h"
#include "ImpactForecast.h"
#include "Logic.h"

void loop()
{
  Log_Init();
  //Blink before starting tests
  for (int i=0;i<8;i++)
  {
    Log_SetLoigcState(i);
    delay(500);
  }
    
  //Testers (uncomment if needed)
  //Log_Test();       //Passed for COM, SD 060317; Flash 120517
  //LED_Test();     //Passed 013018
  //IMU_Test();     //Passed 060317
  //Dist_Test();    //Passed 060317
  //Or_Prop_Test(); //Written (072717) but Not Passed Yet
  //IMFO_Test();    //Passed ImpactTime 060417, Passed IMFO_WhenToStartMotor 022718
  //Motor_Test();     //Written (071917) but Not Passed Yet
  Motor_TestMeasureCapacitorDriverInputVoltage();
  //ls_TestMomentOfInertia(); //Tests moment of inertia by running the motor once fall is detected
  //ls_TestImpactTime(); //This test records distance to flour and impact time


  while (true);
}

#endif
