/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-21
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
	#ifdef WAIT_FOR_SERIAL
	while (! Serial); // Wait for user to open the com port
	#endif
	delay(1000);
	Serial.println("---------------------------------");

	// Declare pins
	// ------------
	pinSetUp();

	// ISRs
	// ----
	attachISRs ();
	enableTrimpots(false); // Disable the trimpot's timer, we will enable them only @ the end of the setup

	// Setting some variables
	// ----------------------
	abortMovement            = false;       
	calibrationSuccess       = false;
	needCalibration          = true;

	// Enabling the stepper
	//---------------------
	Serial.println("Make sure the manual \"stepper enable\" toggle switch on the front panel is set to the \"ENABLED\"");
	delay(1000); 
	Serial.println(" /!\\ Be careful, we are going to enable the stepper");
	delay(1000);  
	enableStepper(true);
	delay(1000); 

	Serial.println("---------------------------------");

	// Usually here you would do the stepper distance calibration but this sketch has not been tested yet as of today (21/07/2021)
	calibrateStepper(); // You need to have enabled the stepper BEFORE

	Serial.println("---------------------------------");

	// Move the shake table back to the center to execute the movement of the trimpots
	moveTableToCenter(); // You need to have enabled the stepper BEFORE
	Serial.println("The shake table should be centered now. You are ready to go!"); 

	Serial.println("---------------------------------");

	// if there are NO errors then we can continue the setup
	if (abortMovement == false)
	{
		/*
		* Make sure the mode recorded in this SW is 
		* the same as the one on the physical toggle switch 
		*/

		displayStepperSettings();
		
		Serial.println("---------------------------------"); // Indicates the end of the setup
		
		enableTrimpots(digitalRead(PIN_TOGGLE_MODE)); // We enable the trimpots here only if we the toggle switch is in manual mode
	}
}







// -------------------------- Loop --------------------------
void loop()
{


	// Checking the flag of some ISRs [3]: LS and Mode select
	//---------------------------------------------------------
	checkISRFlags();
	
	if (abortMovement == false) // the 1st check of this variable out of many more
	{

		// Serial.printf("Current mode: %d | %d\r\n", digitalRead(PIN_TOGGLE_MODE), MODE_MANUAL); 

		switch(digitalRead(PIN_TOGGLE_MODE))
		{
      //-----------------------------------------------------------------------------------
		case MODE_MANUAL :

			
			// Convert trimpot values to stepper parameters
			//----------------------------------------------
			long  halfAmplitudeMicroSteps       = (long)(0.5 * (current_trimpotAmplitude_filtered * ustepsPerMM_calib)); 
			float manual_MicroStepsPerSeconds   = (float)( (float)halfAmplitudeMicroSteps * 2 * current_trimpotFrequency_filtered);


			// Before executing a movement, check that the orders from the trimpots make sense, ie > 0?
			if ( (halfAmplitudeMicroSteps > ((long)0)) && (manual_MicroStepsPerSeconds > 0.0) )
			{
				Serial.printf("Current trimpots %f \t %f \t manual settings: %lu \t %f\r\n", current_trimpotAmplitude_filtered, current_trimpotFrequency_filtered, halfAmplitudeMicroSteps, manual_MicroStepsPerSeconds);

				// Set the maximum allowed speed for manual control (idenpendant of what can be set by the trimpots 
				stepper.setMaxSpeed(manualSpeedMicroStepsPerSeconds);
				stepper.setAcceleration(manualSpeedMicroStepsPerSecondsPerSeconds);
				
				for (int i = 0; i< singleCycleRepetition; i++)
				{

					// Single cycle stepper movement
					//------------------------------
					singleCyleMovement(halfAmplitudeMicroSteps, manual_MicroStepsPerSeconds);

				}

				
				// Here we should be where we started: at the center, ready to start an oscillation again

			}
			else if(stepper.currentPosition () != 0)
			{
				Serial.println("Oups, looks like we didn't stop on the center of the rail, centering");
				moveTableToCenter(); // You need to have enabled the stepper BEFORE
			}
			else
			{
				delay(1);
			}
			
			break;

    //-----------------------------------------------------------------------------------    
		case MODE_SCENARIO :
			// check if a scenario is not already running
			if (executingScenario == false)
			{
				Serial.println("Starting a new Scenario");


        Serial.println("Let's check if the table is centered");        
        if(stepper.currentPosition () != 0)
        {
          Serial.println("Oups, looks like we didn't stop on the center of the rail, centering now");
          moveTableToCenter(); // You need to have enabled the stepper BEFORE
        }

				// <DEBUG> <PLACEHOLDER>
			}
			else // Then you can start a new scenario
			{
				Serial.print("Starting a new scenario...");
				executingScenario = true;
				Serial.println("done");
				// <DEBUG> <PLACEHOLDER>
			}

			break;
			default :
			Serial.println("Impossible mode selected, it must be either Manual or Scenario");
		}// END SWITCH manual/scenario
	} // if abort movement is false
	else
	{
		// If there has been an error, make sure a scenario restart at the its beginning
		executingScenario = false;
	}
}// END OF THE LOOP


// END OF THE FILE
