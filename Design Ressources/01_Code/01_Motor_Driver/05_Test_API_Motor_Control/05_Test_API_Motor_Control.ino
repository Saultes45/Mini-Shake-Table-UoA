/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-22
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

//#include <math.h>  signum


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


void printStepperState(void)
{
  //Do some Set/Get parameter verification here
  Serial.println("*************************************************");
  Serial.println("Let's check some parameters, shall we?");
  Serial.printf("maxSpeed (float): %f \r\n", stepper.maxSpeed());
  Serial.printf("speed (float): %f \r\n", stepper.speed());
  Serial.printf("distanceToGo (signed long): %ld \r\n", stepper.distanceToGo());
  Serial.printf("targetPosition (signed long): %ld \r\n", stepper.targetPosition());
  Serial.printf("currentPosition (signed long): %ld \r\n", stepper.currentPosition());
  Serial.printf("isRunning (boolean): %d \r\n", stepper.isRunning());
  Serial.println("*************************************************");
} // END OF THE FUNCTION



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


  stepper.setCurrentPosition(10);
  printStepperState();
  stepper.setCurrentPosition(-2563);
  printStepperState();
  stepper.setCurrentPosition(0);

  stepper.setMaxSpeed(84000.0);
  
	// Enabling the stepper
	enableStepper(true);
	delay(1000); 
	
	Serial.println("---------------------------------");

}







// -------------------------- Loop --------------------------
void loop()
{

//**********************************************************************
  // Do a constant speed movement + and then -
  
  stepper.moveTo( (long)(+1 * 200) );
  
  stepper.setSpeed(1225.0);
  printStepperState();
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  //while ( (stepper.isRunning()) && (abortMovement == false) )
	{
		stepper.runSpeed();
	}
 //stepper.stop();

  //printStepperState();
  delay(1000);
  
  
  stepper.moveTo( (long)(-1 * 200) );
  stepper.setSpeed(-1225.0);
  printStepperState();
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
	{
		stepper.runSpeed();
	}


  delay(1000);
  stepper.moveTo( (long)(0) );
  Serial.print("Distance to go: ");
  Serial.println(stepper.distanceToGo());
  Serial.print("Sign of DTG ");
  Serial.println((stepper.distanceToGo()>0));
  Serial.print("Speed ");
  Serial.println(((stepper.distanceToGo() > 0) ? +1.0 : -1.0));
  Serial.print("Speed ");
  Serial.println( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * 1225.0 );
  
  stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * 500.0  );
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  {
    stepper.runSpeed();
  }

delay(3000);

//**********************************************************************
  // Do an acceleration movement + and then -
  stepper.moveTo( (long)(+1 * 200*2*10) );
  stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
  
	while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
	{
		stepper.run();
	}

  delay(500);

  stepper.moveTo( (long)(-1 * 200*2*10) );
  stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
  
	while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
	{
		stepper.run();
	}


  delay(1000);
  stepper.moveTo( (long)(0) );
  stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * 500.0  );
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  {
    stepper.runSpeed();
  }

  delay(3000);

  //printStepperState();

//**********************************************************************
//Short accelerations

for (int i = 0; i<10; i++)
{
  stepper.moveTo( (long)(+1 * 200*2*0.25) );
  stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
  
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  {
    stepper.run();
  }
  stepper.moveTo( (long)(-1 * 200*2*0.25) );
  stepper.setAcceleration(84000.0 * 3.5); // reach max speed in 5.0s
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
  delay(6000);


}


// END OF THE FILE
