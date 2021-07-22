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
	while (! Serial); // Wait for user to open the com port
	delay(1000);
	Serial.println("---------------------------------");

	// Declare pins
	// ------------
	pinSetUp();


	// ISRs
	// ----
	attachISRs ();
	enableTrimpots(false); // <-- attachISRs function must be called before this 

	/* After this attachInterrupt, be careful not to toggle the
*  mode switch of the timers or the trimpots will never start
*/

	// Setting some variables
	// ----------------------
	abortMovement            = false;       
	calibrationSuccess       = false;      // A variable that tells if the calibration was sucessful
	needCalibration          = true;       // Indicates if there is a current good distance calibration done


	Serial.println("Make sure the manual \"stepper enable\" toggle switch on the front panel is set to the \"ENABLED\" ");
	delay(1000); 
	Serial.println(" /!\\ Be careful, we are going to enable the stepper");
	delay(1000);  

	// Enabling the stepper
	enableStepper(true);
	delay(1000); 

	// Usually here you would do the stepper distance calibration but this sketch has not been tested yest as of today (21/07/2021)
	calibrateStepper(); // You need to hav enabled the stepper BEFORE

	// Move the shake table back to the center to execute the movement of the trimpots
	moveTableToCenter(); // You need to hav enabled the stepper BEFORE
	Serial.println("The shake table should be centered now. You are ready to go!"); 

	// if ther are no errors then we can continue
	if (abortMovement == false)
	{
		/*
	* Make sure the mode recorded in this SW is 
	* the same as the one on the physical toggle switch 
	*/
		void displayStepperSettings(void);
		Serial.println("---------------------------------"); // Indicates the end of the setup
		enableTrimpots(digitalRead(PIN_TOGGLE_MODE)); 
	}


	// Enable the flag virtually, just once at the start as to read the location of the toggle switch
	//flagMode = true; // <DEBUG> do we still use that?  

}







// -------------------------- Loop --------------------------
void loop()
{


	// Checking the flag of some ISRs [3]: LS and Mode select
	//---------------------------------------------------------
	if ( (flagLS_r == true) || (flagLS_l == true) )
	{
		if (flagLS_r)
		{
			flagLS_r = false;
			Serial.printf("RIGHT Limit Switch ISR triggered %d, current status: %d\r\n", millis(), stateLS_r);
		}
		if (flagLS_l)
		{
			flagLS_l = false;
			Serial.printf("LEFT Limit Switch ISR triggered %d, current status: %d\r\n", millis(), stateLS_l);
		}
	}
	//  else if (flagMode)
	//  {
	//    flagMode = false; // Reset the flag immediatly
	//
	//    // Since the toggle switch ISR is triggered on
	//    //  change, read the final and debounced state
	//    if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
	//    {
	//      // If we are in Manual, then attach the timer interrupt to read the trimpots
	//      enableTrimpots(true);
	//    }
	//    else
	//    {
	//      // If we are in Auto or (Scenario), then detach the
	//      //  timer interrupt to stop reading the trimpots
	//      enableTrimpots(false);
	//    }
	//  }// if (flagMode)

	// Single cycle stepper movement
	//------------------------------

	// if therw are no errors then we can continue and do the movement. This is a BLOCKING call
	if ( abortMovement == false && (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL) )
	{
		Serial.printf("Current mode: %d | %d\r\n", digitalRead(PIN_TOGGLE_MODE), MODE_MANUAL); 
		
		long 	halfAmplitudeMicroSteps 		= (long)(0.5 * (current_trimpotAmplitude_filtered * ustepsPerMM_calib)); 
		float 	manual_MicroStepsPerSeconds 	= (float)( (float)halfAmplitudeMicroSteps * current_trimpotFrequency_filtered );

		// Before executing a movement, check that the orders from the trimpots make sense, ie > 0?
		if ( (halfAmplitudeMicroSteps > ((long)0)) && (manual_MicroStepsPerSeconds > 0.0) )
		{
			Serial.printf("Current trimpot values: %f | %f\r\n", current_trimpotAmplitude_filtered, current_trimpotFrequency_filtered);
			Serial.printf("Current user manual settings: %lu | %f\r\n", halfAmplitudeMicroSteps, manual_MicroStepsPerSeconds);

			
			// apply to the motor

			stepper.setSpeed(manual_MicroStepsPerSeconds); // Order matters!!!! 1-> max speed (needed for accel) 2->accel 3->target pos (steps)
			//stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds); // <DEBUG> No acceleration

			// 1st movement -1/2
			//-------------------
			stepper.moveTo( (long)(+1 * halfAmplitudeMicroSteps) );
			while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
			{
				stepper.run();
			}

			// 2nd movement +1 
			//-------------------
			//    stepper.moveTo( (long)(+2 * halfAmplitudeMicroSteps) );
			//    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
			//    {
			//      stepper.run();
			//    }
			

			// 3rd movement -1/2
			//-------------------
			stepper.moveTo( (long)(+1 * halfAmplitudeMicroSteps) );
			while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
			{
				stepper.run();
			}

			// Here we should be where we started: at the center, ready to start an oscillation again

		}
		
	}

}


// END OF THE FILE
