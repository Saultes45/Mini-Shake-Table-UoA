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
// moved in "Global.h"

// -------------------------- Global variables ----------------
// moved in "Global.h"

// -------------------------- Functions declaration [7] --------------------------
// moved in "Global.h"





// -------------------------- SetUp --------------------------
void setup()
{

	// Debug communication channel UART through USB-C
	// -----------------------------------------------
	Serial.begin(CONSOLE_BAUD_RATE); //Begin serial communication (USB)
	#ifdef WAIT_FOR_SERIAL
	while (! Serial); // Wait for user to open the com port // <DEBUG> THIS IS ULATRA DANGEROUS
	#endif
	delay(1000);
	Serial.println("---------------------------------");

	// Declare pins
	// ------------
	pinSetUp();

  // Setting some variables
	// ----------------------
	abortMovement            = false;       
	calibrationSuccess       = false;
	needCalibration          = true;
  executingCalib           = true; // This must absolutely be done BEFORE the ISRs // <DEBUG> might be redundant

	// ISRs
	// ----
	attachISRs();
	enableTrimpots(false); // Disable the trimpot's timer, we will enable them only @ the end of the setup

	// Enabling the stepper
	//---------------------
	Serial.println("Make sure the manual \"stepper enable\" toggle switch on the front panel is set to the \"ENABLED\"");
	delay(1000); 
	Serial.println("/!\\ We are going to ENABLE the stepper, be careful");
	delay(1000);  
	enableStepper(true);
	delay(1000); 

	Serial.println("---------------------------------");

	calibrateStepper(); // You need to have enabled the stepper BEFORE

	Serial.println("---------------------------------");
	
  // if there are NO errors then we can continue the setup
	if (abortMovement == false)
	{
    // Move the shake table back to the center to execute the movement of the trimpots
    delay(5000); // This is to be able to tell the difference between fake calibration and move to center
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

      // Start the serial port with the Rapsberry Pi (both HW)
      /* You can use Python "import serial"
      *  OR you can use the commad line with existing tools: stty, minicom, https://wiki.emacinc.com/wiki/Getting_Started_With_Minicom
      *  supported by stty: 230400, 460800, 500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000 and 4000000.
      *  look at that solution for 921600: Ctrl+F --> "Wed Apr 02, 2014 4:46 pm" in: https://www.raspberrypi.org/forums/viewtopic.php?t=73673
      *  port = "/dev/ttyAMA0"    # Raspberry Pi 2
      *  port = "/dev/ttyS0"      # Raspberry Pi 3
      */
      Serial1.begin(COMMAND_BAUD_RATE);
      flushReceiveAndTransmit();
  
      
      Serial.println("---------------------------------"); // Indicates the end of the setup
      
      enableTrimpots(digitalRead(PIN_TOGGLE_MODE)); // We enable the trimpots here only if we the toggle switch is in manual mode
    }
	}
}







// -------------------------- Loop --------------------------
void loop()
{


	// 1/3 - Checking the flag of some ISRs [3]: LS [2] and Mode Select [1]
	//---------------------------------------------------------------
	checkISRFlags();

  // 2/3 - Checking if we have received data from the RPi serial port
	//-----------------------------------------------------------
	readRPiBuffer();
	
  // 3/3 - Checking if we need to perform a movement with the stepper
	//-----------------------------------------------------------
	if (abortMovement == false) // the 1st check of this variable out of many more
	{

		// Serial.printf("Current mode: %d | %d\r\n", digitalRead(PIN_TOGGLE_MODE), MODE_MANUAL); 

		switch(digitalRead(PIN_TOGGLE_MODE))
		{
      //-----------------------------------------------------------------------------------
		case MODE_MANUAL:
    {

			// Convert trimpot values to stepper parameters
			//----------------------------------------------
			long  halfAmplitudeMicroSteps       = (long)(0.5 * (current_trimpotAmplitude_filtered * ustepsPerMM_calib)); 
			float manual_MicroStepsPerSeconds   = (float)( (float)halfAmplitudeMicroSteps * 2 * current_trimpotFrequency_filtered);

      Serial.printf("Current trimpots %f \t %f \t Manual settings: %lu \t %f\r\n", current_trimpotAmplitude_filtered, current_trimpotFrequency_filtered, halfAmplitudeMicroSteps, manual_MicroStepsPerSeconds);


			// Before executing a movement, check that the orders from the trimpots make sense, i.e. > 0?
			if ( (halfAmplitudeMicroSteps > ((long)0)) && (manual_MicroStepsPerSeconds > 0.0) && (abortMovement == false))
			{

				// Set the maximum allowed speed for manual control (idenpendant of what can be set by the trimpots 
				stepper.setMaxSpeed(manualSpeedMicroStepsPerSeconds);
				stepper.setAcceleration(manualSpeedMicroStepsPerSecondsPerSeconds);
				
				//for (int i = 0; i < singleCycleRepetition; i++)
				//{

					// Single cycle stepper movement
					//------------------------------
					singleCyleMovement(halfAmplitudeMicroSteps, manual_MicroStepsPerSeconds);

				//}

				// Here we should be where we started: @ the center, ready to start an oscillation again

			}
			else if(stepper.currentPosition () != 0)
			{
				Serial.println("Oups, looks like we didn't stop on the center of the rail, centering now");
				moveTableToCenter(); // You need to have enabled the stepper BEFORE
			}
			else
			{
				delay(250); // wait a bit for another action from the user, like > 0 trimpots or mode change 
			}
			
			break;
   }

    //-----------------------------------------------------------------------------------    
		case MODE_SCENARIO:
    {
			// check if a scenario is not already running
      // Step 1 of 2 for scenario
			if ( not(executingScenario) && needScenario && (abortMovement == false) )
			{
        prepareScenario();
			}
      // Step 2 of 2 for scenario
			else if ( (executingScenario) && not(needScenario) && (abortMovement == false) )// Then you execute the scenario that has been prepared
			{
				executeScenario();
			}

			break;
   }
			default:
      {
			Serial.println("Impossible mode selected, it must be either Manual or Scenario");
//     break;
      }
		}// END SWITCH manual/scenario
	} // if abort movement is false
	else
	{
		// If there has been an error, make sure a scenario restart at the its beginning
		executingScenario = false;
	}
}// END OF THE LOOP



// END OF THE FILE
