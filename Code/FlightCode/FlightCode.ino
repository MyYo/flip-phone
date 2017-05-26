////////////////////////////////////////////////////////////////////////////////
// Don't forget to check that all pins are assigned correctly (they currently
// are not) and to set the experiment parameters (they currently are not).
// Better yet, actually look through the code (especially the early variable
// assignment parts) to make sure that things make sense before bricking your 
// device!
////////////////////////////////////////////////////////////////////////////////
//General Includes

#include "HardwareConfiguration.h"
#include "Logger.h"
#include "Logic.h"

void setup() 
{
  Log_Init();
}

void loop() 
{
  //Operational mode
  RunLogic();

  //Testers (uncomment if needed)
  //Log_Test();     //Not Passed Yet
  //IMU_Test();     //Not Passed Yet
  //Dist_Test();    //Not Passed Yet
  //Or_Prop_Test(); //Not Passed Yet
  //IMFO_Test();    //Not Passed Yet
  //Eng_Test();     //Not Passed Yet
}
