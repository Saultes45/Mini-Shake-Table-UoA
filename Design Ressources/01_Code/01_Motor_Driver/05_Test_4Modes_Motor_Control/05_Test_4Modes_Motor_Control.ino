/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-23
* Version       : 0.1 (finished on 2021-..-..)
* Modifications :
* Known bugs    :
*
*
* Possible Improvements
* Remove deadband from the ADC range in the conversion
* Check for #ifdef SERIAL_VERBOSE where there is a "Serial.print..."
*
* Notes
*
*
* Ressources (Boards + Libraries Manager)
*
*
* TODO
*
* ========================================
*/


// -------------------------- Includes --------------------------
#include "Global.h"

// -------------------------- Defines --------------------------

// Stepper motor driver pins (logic low)
//--------------------------------------
//moved in "Stepper.h"

// Stepper conversion factors
//-----------------------------
//moved in "Stepper.h"

// Limit switches pins
//--------------------
//moved in "LimitSwitches.h"

// Trimpot pins
//--------------
//moved in "Trimpot.h"

// Mode selection
//---------------
//moved in "Global.h"


// -------------------------- Global variables ----------------

// Limit switches
//---------------
//moved in "LimitSwitches.h"

// Mode toggle switch
//-------------------
//moved in "Global.h"

// Trimpots
//---------
// moved in "Trimpot.h"


// Stepper motor driver 
//----------------------
//moved in "Stepper.h"


// Stepper calibration
//---------------------
// moved in in "StepperCalibration.h"

// -------------------------- Functions declaration [7] --------------------------
// moved in "Global.h"



// -------------------------- SetUp --------------------------
void setup()
{

	// Debug communication channel UART through USB-C
	// -----------------------------------------------
	Serial.begin(CONSOLE_BAUD_RATE); //Begin serial communication (USB)
	while (! Serial); // Wait for user to open the com port
	delay(1000);
	Serial.println("---------------------------------");

	// Declare pins
	// ------------
	pinSetUp();

  abortMovement = false;
  
	// Enabling the stepper
	enableStepper(true);
	delay(500); 
	
	Serial.println("---------------------------------");

}



// -------------------------- Loop --------------------------
void loop()
{

  const int   nbr_movementsScenario           = 10; // This variable is const because it defuines the size of the memory we can allocate to scenario parameters
  long  scenarioSteps[nbr_movementsScenario]  = {0};
  float scenarioSpeed[nbr_movementsScenario]  = {0.0};
  bool  abortScenario                         = false; // a variable that store potential reason to stop a scenario


  scenarioSteps[0] = +200; // the first one is always 1/2, I don't know why
  scenarioSteps[1] = -200;
  scenarioSteps[2] = +200;
  scenarioSteps[3] = -100;
  scenarioSteps[4] = +100;
  scenarioSteps[5] = +50;
  scenarioSteps[6] = -25;
  scenarioSteps[7] = +25;
  scenarioSteps[8] = -200;
  scenarioSteps[9] = +200;


  scenarioSpeed[0] = +250.0;
  scenarioSpeed[1] = -10000.0;
  scenarioSpeed[2] = +10000.0;
  scenarioSpeed[3] = -10000.0;
  scenarioSpeed[4] = +10000.0;
  scenarioSpeed[5] = +10000.0;
  scenarioSpeed[6] = -10000.0;
  scenarioSpeed[7] = +1000.0;
  scenarioSpeed[8] = -500.0;
  scenarioSpeed[9] = +25.0;

  //**********************************************************************
  // Mode 1 - Homing/Calibration/Detriggering - no acceleration, slow speed


  Serial.println("Mode 1 - Homing/Calibration/Detriggering - no acceleration, VERY slow speed");
  calibrateStepper();
  
  // printStepperState();

  // Wait before running next mode
  delay(3000);

  //**********************************************************************
  // Mode 2 - Centering - no acceleration, slow speed

  Serial.println("Mode 2 - Centering - no acceleration, slow speed");
  moveTableToCenter();

  //printStepperState();

  // Wait before running next mode
  delay(3000);

  //**********************************************************************
  // Mode 3 - Single cycle movement - speed over acceleration, we DO need acceleration


  Serial.println("Mode 3 - Single cycle movement - speed over acceleration, we DO need acceleration");

  stepper.setMaxSpeed(84000.0);


  for (int i = 0; i<10; i++)
  {
    stepper.moveTo( (long)(+1 * 200*2*0.25) );
    stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
    
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      stepper.run();
    }
    stepper.moveTo( (long)(-1 * 200*2*0.25) );
    //stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      stepper.run();
    }
  }


  delay(1000);
  stepper.moveTo( (long)(0) );
  stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * 500.0  );
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  {
    stepper.runSpeed();
  }

  // Wait before running next mode
  delay(3000);

  //**********************************************************************
  // Mode 4 - Scenario - accel is needed but it must be time accurate

  Serial.println("Mode 4 - Scenario - accel is needed but it must be time accurate");

   for (int cnt_scenarioMovements = 0 ; cnt_scenarioMovements < nbr_movementsScenario; cnt_scenarioMovements++)
    {
      stepper.setSpeed(scenarioSpeed[cnt_scenarioMovements]); // Order matters!!!! 1->speed 2->steps
      stepper.moveTo(scenarioSteps[cnt_scenarioMovements]);
      while ((stepper.distanceToGo() != 0) && (abortScenario == false))
      {
        stepper.run();
      }
    }


   // Wait before running next cycle
  delay(6000);


}


// END OF THE FILE
