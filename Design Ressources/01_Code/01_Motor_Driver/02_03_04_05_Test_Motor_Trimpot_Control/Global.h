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

// Personal libraries
//--------------------
#include "StepperCalibration.h"
#include "LimitSwitches.h"
#include "Trimpot.h"
#include "Stepper.h"
#include "Scenario.h"


// -------------------------- Defines --------------------------
// General
#define SERIAL_VERBOSE                        // Uncomment to see more debug messages
//#define WAIT_FOR_SERIAL                       // Uncomment to wait for the serial port to be opened from the PC end before starting
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console
//#define SHOW_TRIMPOT_VALUE                  // Uncomment to see the calculated frequency and amplitude values in the ISR


// Mode selection
//---------------
const uint8_t PIN_TOGGLE_MODE           = 8;
const uint16_t MS_DEBOUNCE_TIME         = 50;      // mode switch button debouncing time in [ms]

// Modes
#define MODE_SCENARIO   0u  // Wait orders and scenarios from the Raspberry pi through raspberry pi 30 pin connector (HW UART on the XIAO)
#define MODE_MANUAL     1u  // Use the 2 trimpots to execute the motion


// -------------------------- Global variables ----------------

// Mode toggle switch
//-------------------
volatile bool           flagMode            = false;
volatile unsigned long  last_interrupt_time = 0;

// -------------------------- Functions declaration [13] --------------------------
void      pinSetUp                (void);
void      attachISRs              (void);
void      enableTrimpots          (bool enableOrder);
void      enableStepper           (bool enableOrder);
void      displayStepperSettings  (void);
void      moveTableToCenter       (void);
void      calibrateStepper        (void);
void      printStepperState       (void);
void      checkISRFlags           (void);
void 	    singleCyleMovement      (long halfAmplitudeMicroSteps, float manual_MicroStepsPerSeconds);
void      readTrimpots            (void);
void      prepareScenario         (void);
void      executeScenario         (void);





// -------------------------- ISR [4] ----------------

//******************************************************************************************
void LS_r_ISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time_ls_r > LS_DEBOUNCE_TIME)
	{
		flagLS_r = true; // This indicates a change
    stateLS_r = digitalRead(PIN_LIMIT_RIGHT); // This is a SATE, not a FLAG!, the signal should be stable enough to read from the pin directly
		
    if(stateLS_r)
		{
			enableStepper(executingCalib); // disable the stepper only if NOT in calibration
      
      // We distinguish 2 cases: in calibration (we expect a LS trigger) and NOT in calibration (no LS trigger expected)
      if(executingCalib)
      {
        // <DEBUG> <DO we have to have this?>
      }
      else
      {
        stepper.stop(); // Although the motor is now disabled, the uController doesn't know that and continues to generate pulses

        enableTrimpots(false);

        //Change some states and some flags
        //--------------------------------
        needCalibration           = true;       // Indicates if there is a current good distance calibration done
        executingCalib            = false;      // Indicates if we are currently in distance calibration with the Limit Switches
        calibrationSuccess        = false;      // A variable that tells if the calibration was sucessful
        abortMovement             = true;
      }
      
      //if there is a need for restart: asm volatile ("  jmp 0");
		}
		// else // here is the detrigger
		// {
		// 	abortMovement = false;
		// }
		last_interrupt_time_ls_r = interrupt_time;
	}

	interrupts();
}

//******************************************************************************************
void LS_l_ISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time_ls_l > LS_DEBOUNCE_TIME)
	{
		flagLS_l = true; // This indicates a change
    stateLS_l = digitalRead(PIN_LIMIT_LEFT); // This is a SATE, not a FLAG!, the signal should be stable enough to read from the pin directly
		
    if(stateLS_l)
		{
			enableStepper(executingCalib); // disable the stepper only if NOT in calibration
      
      // We distinguish 2 cases: in calibration (we expect a LS trigger) and NOT in calibration (no LS trigger expected)
      if(executingCalib)
      {
        // <DEBUG> <DO we have to have this?>
      }
      else
      {
        stepper.stop(); // Although the motor is now disabled, the uController doesn't know that and continues to generate pulses

        enableTrimpots(false);

        //Change some states and some flags
        //--------------------------------
        needCalibration           = true;       // Indicates if there is a current good distance calibration done
        executingCalib            = false;      // Indicates if we are currently in distance calibration with the Limit Switches
        calibrationSuccess        = false;      // A variable that tells if the calibration was sucessful
        abortMovement             = true;
      }
      
      //if there is a need for restart: asm volatile ("  jmp 0");
		}
		// else // here is the detrigger
		// {
		// 	abortMovement = false;
		// }
		last_interrupt_time_ls_l = interrupt_time;
	}

	interrupts();
}

//******************************************************************************************
void toggleSwitchModeISR()
{
	//noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > MS_DEBOUNCE_TIME)
	{
		//flagMode = true;
    needScenario  		  = false;
    executingScenario 	= false;

		if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
		{
			// If we are in Manual, then attach the timer interrupt to read the trimpots
			enableTrimpots(true);
      
		}
		else
		{
			// If we are in Scenario mode, then detach the
			//  timer interrupt to stop reading the trimpots
			enableTrimpots(false);
			// And set a flag to ask to execute a scenario
      needScenario = true;
		}
		
	}
	last_interrupt_time = interrupt_time;
	//interrupts();
}

//******************************************************************************************
void timerTrimpotISR()
{
	// 1st check if we are in manual mode because scenario mode doesn't use
	if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
	{
		trimpotFlag = true;// set the flag
	}

	/* NOTICE:
*  Do NOT put noInterupts(); here because this is a low priority and long execution time "task"
*  Limit switches IRS should be able to trigger in there
*/

}


/*------------------------------------------------------*/
/*-------------------- Functions [9] -------------------*/
/*------------------------------------------------------*/

//******************************************************************************************
void enableTrimpots(bool enableOrder)
{
	if (enableOrder)
	{
		#ifdef SERIAL_VERBOSE
		Serial.print("ENABLING the TRIMPOTS...");
		#endif

		#ifdef USE_TC3
		TimerTc3.initialize(FREQUENCY_READ_TRIMPOTS);
		TimerTc3.attachInterrupt(timerTrimpotISR);
		#else
		TimerTcc0.initialize(FREQUENCY_READ_TRIMPOTS); // 1e6 = 1s
		TimerTcc0.attachInterrupt(timerTrimpotISR);
		#endif

	}
	else
	{
		#ifdef SERIAL_VERBOSE
		Serial.print("DISABLING the TRIMPOTS...");
		#endif

		#ifdef USE_TC3
		TimerTc3.initialize(FREQUENCY_READ_TRIMPOTS);
		TimerTc3.detachInterrupt();
		#else
		TimerTcc0.initialize(FREQUENCY_READ_TRIMPOTS); // 1e6 = 1s
		TimerTcc0.detachInterrupt();
		#endif

	}
#ifdef SERIAL_VERBOSE
	Serial.println("done");
#endif

}// END OF THE FUNCTION

//******************************************************************************************
void attachISRs (void)
{

#ifdef SERIAL_VERBOSE
	Serial.print("Attaching interrupts to their functions, be careful not to trigger anything...");
#endif
	/* Attach each LS a different ISR
*  For each we want to know when it is
*  triggered and released so we can enable the drive again if necesary
*/
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LS_r_ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LS_l_ISR, CHANGE);

  // Reset the time when the last LS interrupt was triggered (reference)
  last_interrupt_time_ls_r = millis();
  last_interrupt_time_ls_l = millis();

	// To trigger an action when the toggle switch associated with the mode selection (Manual/Scenario)
	attachInterrupt(digitalPinToInterrupt(PIN_TOGGLE_MODE), toggleSwitchModeISR, CHANGE); //toggleSwitchModeISR

#ifdef SERIAL_VERBOSE
	delay(250);
	Serial.println("done");
#endif

}// END OF THE FUNCTION

//******************************************************************************************
void pinSetUp (void)
{

#ifdef SERIAL_VERBOSE
	Serial.print("Setting up the pins...");
#endif

	// initialize digital pin PIN_MOTOR_ENA as an output
	pinMode(PIN_MOTOR_ENA, OUTPUT);

	// disable the motor immediatly (it is alseo recommended to have the external toggle switch in the "disable" positon)
	enableStepper(false);

	// initialize the LED pin as an output:
	pinMode(LED_BUILTIN, OUTPUT);

	// initialize the limit switches pins as an input
	pinMode(PIN_LIMIT_RIGHT, INPUT);
	pinMode(PIN_LIMIT_LEFT,  INPUT);

	// initialize the pushbutton pin as an input
	pinMode(PIN_TOGGLE_MODE, INPUT);

#ifdef SERIAL_VERBOSE
	Serial.println("done");
#endif

}// END OF THE FUNCTION

//******************************************************************************************
void enableStepper(bool enableOrder)
{
	if (enableOrder)
	{
		if ( (digitalRead(PIN_LIMIT_RIGHT) == false) && (digitalRead(PIN_LIMIT_LEFT) == false) && (abortMovement == false) )
		{
			#ifdef SERIAL_VERBOSE
			Serial.println("ENABLING the stepper, be careful...");
			#endif
			digitalWrite(PIN_MOTOR_ENA,LOW); // Inverse logic (active low)
			//    digitalWrite(PIN_MOTOR_ENA,HIGH); // Normal logic (active high)
		}
	}
	else // in that case we want to be BLAZING FAST -> digitalWrite is BEFORE the Serial and TODO: used DMA instead
	{
		digitalWrite(PIN_MOTOR_ENA,HIGH); // Inverse logic (active low)
		//    digitalWrite(PIN_MOTOR_ENA,LOW); // Normal logic (active high)
		#ifdef SERIAL_VERBOSE
		Serial.println("You are going to lose the reference position, calibration will be required");
		Serial.print("DISABLING the stepper...");
		#endif
		needCalibration       = true;
		calibrationSuccess    = false;      // A variable that tells if the calibration was sucessful

		#ifdef SERIAL_VERBOSE
		Serial.println("done");
		#endif
	}

}// END OF THE FUNCTION

//******************************************************************************************
void displayStepperSettings(void)
{
	// Display settings to UART
	//-------------------------
	Serial.println("---------------------------------");

	Serial.print("Current micro-steps settings: ");
	Serial.println(microSteppingFactorList[indx_microSteppingFactorList]);

	Serial.print("Current micro-steps per rotation: ");
	Serial.println(microstepsPerRevolution);

	Serial.println("---------------------------------");

	// To calculate max acceleration, max speed must be set first

	Serial.print("Current max speed: ");
	Serial.println(stepper.maxSpeed ()); // Returns the most recent speed in steps per second
	Serial.print("Setting max speed to: ");
	Serial.println(max_allowedMicroStepsPerSeconds); // Returns the most recent speed in [steps per second]
	stepper.setMaxSpeed(max_allowedMicroStepsPerSeconds);
	Serial.print("New max speed is: ");
	Serial.println(stepper.maxSpeed ()); // Returns the most recent speed in steps per second (float)

	Serial.print("Setting acceleration to: ");
	Serial.println(max_allowedMicroStepsPerSecondsPerSeconds);
	stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds);
	// No function to read back set acceleration
	// NOTICE: Calling setAcceleration() is expensive, since it requires a square root to be calculated.

}// END OF THE FUNCTION

//******************************************************************************************
void  moveTableToCenter   (void)
{

  #ifdef SERIAL_VERBOSE
    Serial.println("Function moveTableToCenter called. For this to work, the stepper must be ALREADY enabled");
    Serial.printf("Moving table back to the center position: %ld [mm] ... ", (long)(distanceBetweenLS_MM/2.0 * ustepsPerMM_calib));
  #endif

  if (abortMovement == false)
  {
    stepper.setMaxSpeed(centeringSpeedMicroStepsPerSeconds_max);
    stepper.setAcceleration(centeringSpeedMicroStepsPerSecondsPerSeconds);
    stepper.moveTo( (long)(0) );
    //stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * centeringSpeedMicroStepsPerSeconds_normal);
    // printStepperState();
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      //stepper.runSpeed();
      stepper.run();
    }

    #ifdef SERIAL_VERBOSE
      Serial.println("done");
    #endif

  }//abortMovement
  else
  {
    Serial.println("Aborting because \"abortMovement\" is set");
  }

}// END OF THE FUNCTION

//******************************************************************************************
void  calibrateStepper (void) // <TODO> This is currently a placeholder, use the code from "05_02_TestCalibartion_StepperDistance_Limitswitches"
{

	/* Theory of operation:
  * Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT
  * This is the START position.
  * Move to the the LEFT until (timeout) or LEFT LS triggers, if the LFT one triggers, ABORT
  */

  #ifdef SERIAL_VERBOSE
    Serial.println("Function calibrateStepper called. For this to work, the stepper must be ALREADY enabled");
  #endif

  if ( (needCalibration == true) && (abortMovement == false) )
  {
    executingCalib = true; // set the boolean state // <DEBUG> might be redundant

    #ifdef SERIAL_VERBOSE
      Serial.println("Starting calibration now");
    #endif


    //* If the table is touching both (2) LSs, ABORT, beacause that should not happen
    if ( not( (digitalRead(PIN_LIMIT_RIGHT) == 1) && (digitalRead(PIN_LIMIT_LEFT) == 1) ) )
    {
      Serial.println("At least 1 of the LS is NOT ON"); // <DEBUG>

      //* If the table is touching ONLY 1 LS, then move the other direction
      while((digitalRead(PIN_LIMIT_RIGHT) == 1) || (digitalRead(PIN_LIMIT_LEFT) == 1))
      {
        Serial.println("At least 1 of the 2 LS is ON, detriggering now"); // <DEBUG>
        // <TODO> take into account the direction
        // The stepper should already be enabled

        // Let's move to the LEFT (-) if RIGHT is triggered: RELATIVE
        stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
        stepper.move( (long)((((digitalRead(PIN_LIMIT_LEFT) == 1)) ? +1.0 : -1.0) * detriggeringExplorationMicroSteps) ); // This RELATIVE!
        stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal);
        // printStepperState();
        while ( (stepper.distanceToGo() != 0) && (abortMovement == false) )
        {
          stepper.runSpeed();
        }

        Serial.println("Detriggereing movement done, was that enough?"); // <DEBUG>
        // Serial.print("abortCalibration:"); // <DEBUG>
        // Serial.println(abortCalibration); // <DEBUG>
      }

      //* Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT
      #ifdef SERIAL_VERBOSE
        Serial.println("Trying to reach the 1st LS: the RIGHT one, waiting for an ISR trigger...");
      #endif

      // Let's move to the RIGHT: RELATIVE
      stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
      stepper.move( (long)(+1 * calibrationExplorationMicroSteps) ); // This RELATIVE!
      stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal);
      // printStepperState();
      unsigned long startedWaiting = millis();
      Serial.println("Started the run "); // <DEBUG>
      while ( (stepper.distanceToGo() != 0) && (abortMovement == false)  && (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS) )
      {
        stepper.runSpeed();
      }
      #ifdef SERIAL_VERBOSE
        Serial.println("Run ended, why?"); // <DEBUG>
      #endif

      // Set the rightmost position TEMPORARY as 0 to make the step counting easy as
      stepper.setCurrentPosition(0);













  

	// // Let's move to the LEFT: RELATIVE
	// stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
	// stepper.move( (long)(-1 * calibrationExplorationMicroSteps) ); // This RELATIVE!
	// stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal );
	// // printStepperState();
	// while ( (stepper.distanceToGo() != 0) && (abortMovement == false) )
	// {
	// 	stepper.runSpeed();
	// }


	// This is the END position, get the table (motor) position and save it (homebrew abs())
	//distanceBetweenLS_uSteps = (stepper.currentPosition() > 0) ? +1.0 : -1.0) * stepper.currentPosition();// This is also a distance since we TEMPORARY marked the 0mm as the right-most LS
	distanceBetweenLS_uSteps = 501; // <DEBUG>

	//* Calculate how many steps have been executed to travel the USER-DEFINED distance between the 2 LS
	ustepsPerMM_calib = distanceBetweenLS_uSteps / distanceBetweenLS_MM;

	// Define where the 0 position is (it is @ the center of the LS)
	stepper.setCurrentPosition( -1 * (long)(distanceBetweenLS_uSteps/2) );

	Serial.println("Calibration success!");
	Serial.printf("The calibrated distance bewteen the 2 limit switches in STEPS is %ld \r\n", distanceBetweenLS_uSteps);
	Serial.printf("The user-defined distance bewteen the 2 limit switches is %f [mm] \r\n", distanceBetweenLS_MM);
	Serial.printf("The calibrated STEPS to MM ratio is: %f [mm/steps]\r\n", ustepsPerMM_calib);

	calibrationSuccess = true;


	if(calibrationSuccess)
	{
		needCalibration          = false;     // Indicates if there is a current good distance calibration done
		#ifdef SERIAL_VERBOSE
		Serial.println("Calibration carried out sucessfully");
		#endif
	}
	else
	{
		needCalibration          = true;
		#ifdef SERIAL_VERBOSE
		Serial.println("Calibration failed, try again");
		#endif
	}

  executingCalib = false; // reset the boolean state

}
  

}// END OF THE FUNCTION

//******************************************************************************************
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


//******************************************************************************************
void readTrimpots(void)
{
	if( (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL) )
	{
		// Local variable declaration
		//----------------------------
		uint16_t sensorValue1 = 0;
		uint16_t sensorValue2 = 0;


		// Read the input on analog pin 0 - Frequency
		//-------------------------------------------
		sensorValue1 = analogRead(PIN_TRMPT_FREQ);
		// Apply deadband
		if (sensorValue1 <= TRMPT_DEADBAND_FREQ)
		{
			// Then we are in the deadband and the outpout value should be 0[LSB]
			sensorValue1 = 0;
		}
		median1 = medianFilter1.AddValue(sensorValue1);
		current_trimpotFrequency_filtered = (median1-trimpot_frequency_range_min_lsb) * (trimpot_frequency_max - trimpot_frequency_min)/(trimpot_frequency_range_max_lsb-trimpot_frequency_range_min_lsb) + trimpot_frequency_min;
		if (current_trimpotFrequency_filtered < trimpot_frequency_min)
		{
			current_trimpotFrequency_filtered = trimpot_frequency_min;
		}
		else if(current_trimpotFrequency_filtered > trimpot_frequency_max)
		{
			current_trimpotFrequency_filtered = trimpot_frequency_max;
		}


		// Read the input on analog pin 1 - Amplitude
		//-----------------------------------------------
		sensorValue2 = analogRead(PIN_TRMPT_AMPL);
		// Apply deadband
		if (sensorValue2 <= TRMPT_DEADBAND_AMPL)
		{
			// Then we are in the deadband and the outpout value should be 0[LSB]
			sensorValue2 = 0;
		}
		median2 = medianFilter2.AddValue(sensorValue2);
		current_trimpotAmplitude_filtered = (median2-trimpot_amplitude_range_min_lsb) * (trimpot_amplitude_max - trimpot_amplitude_min)/(trimpot_amplitude_range_max_lsb-trimpot_amplitude_range_min_lsb) + trimpot_amplitude_min;
		if (current_trimpotAmplitude_filtered < trimpot_amplitude_min)
		{
			current_trimpotAmplitude_filtered = trimpot_amplitude_min;
		}
		else if(current_trimpotAmplitude_filtered > trimpot_amplitude_max)
		{
			current_trimpotAmplitude_filtered = trimpot_amplitude_max;
		}


		// Display (for  <DEBUG> only)
		//----------------------------
    #ifdef SHOW_TRIMPOT_VALUE
		// Raw values
		 Serial.print(sensorValue1);
		 Serial.print(" ");
		 Serial.print(median1);
		 Serial.print(" ");
		 Serial.print(sensorValue2);
		 Serial.print(" ");
		 Serial.print(median2);
     Serial.print(" ");
    //  Serial.println();
		// Final values
		Serial.print  (current_trimpotFrequency_filtered);
		Serial.print(" ");
		Serial.println(current_trimpotAmplitude_filtered);
    #endif
		
	}
}// END OF THE FUNCTION


//******************************************************************************************
void checkISRFlags(void)
{

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


	if( trimpotFlag )
	{
		trimpotFlag = false;// reset the flag
		readTrimpots();
	}

} // END OF THE FUNCTION


//******************************************************************************************
void singleCyleMovement(long  halfAmplitudeMicroSteps, float manual_MicroStepsPerSeconds)
{
	// 1st movement -1/2
	//-------------------
  if (abortMovement == false) 
  {
    stepper.move( (long)(-1 * halfAmplitudeMicroSteps) );
    stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * manual_MicroStepsPerSeconds);
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      stepper.run();
    }
  }


	// 2nd movement +1
	//-------------------
  if (abortMovement == false) 
  {
    stepper.move( (long)(+2 * halfAmplitudeMicroSteps) );
    stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * manual_MicroStepsPerSeconds);
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      stepper.run();
    }
  }

	

	// 3rd movement -1/2
	//-------------------
  if (abortMovement == false) 
  {
    stepper.move( (long)(-1 * halfAmplitudeMicroSteps) );
    stepper.setSpeed( ((stepper.distanceToGo() > 0) ? +1.0 : -1.0)  * manual_MicroStepsPerSeconds);
    while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
    {
      stepper.run();
    }
  }
	
}// END OF THE FUNCTION

//******************************************************************************************
void prepareScenario(void)
{
  needScenario = false; // reset the flag
        
  Serial.println("Starting a new scenario");


  Serial.println("Let's check if the table is centered");
  if( (stepper.currentPosition() != 0) && (abortMovement == false) )
  {
    Serial.println("Oups, looks like we didn't stop on the center of the rail, centering now");
    moveTableToCenter(); // You need to have enabled the stepper BEFORE
  }

  Serial.print("cleaning previous scenario data...");
  memset (scenarioSteps, (long)0 ,    nbr_movementsScenario);
  memset (scenarioSpeed, (float)0.0 , nbr_movementsScenario);
  Serial.println("done");

  Serial.print("Loading scenario data...");
  // Amplitude in [microsteps]
  scenarioSteps[0] = +200;
  scenarioSteps[1] = -200;
  scenarioSteps[2] = +200;
  scenarioSteps[3] = -100;
  scenarioSteps[4] = +100;
  scenarioSteps[5] = +50;
  scenarioSteps[6] = -25;
  scenarioSteps[7] = +25;
  scenarioSteps[8] = -200;
  scenarioSteps[9] = +200;

  // Speeds must all be > 0, analog to frequency, in [microsteps/s]
  // Have you noticed the ".0"? This is MANDATORY
  scenarioSpeed[0] = 250.0;
  scenarioSpeed[1] = 1000.0;
  scenarioSpeed[2] = 850.0;
  scenarioSpeed[3] = 360.0;
  scenarioSpeed[4] = 400.0;
  scenarioSpeed[5] = 1000.0;
  scenarioSpeed[6] = 900.0;
  scenarioSpeed[7] = 914.0;
  scenarioSpeed[8] = 500.0;
  scenarioSpeed[9] = 100.0;
  Serial.println("done");

  // Set the maximum allowed speed for manual control (idenpendant of what can be set by the trimpots
  Serial.print("Setting the maximum allowed speed for scenario settings..."); 
  stepper.setMaxSpeed(manualSpeedMicroStepsPerSeconds);
  stepper.setAcceleration(manualSpeedMicroStepsPerSecondsPerSeconds);
  Serial.println("done");

  executingScenario = true; // setting the boolean for the next iteration

}// END OF THE FUNCTION

//******************************************************************************************
void executeScenario(void)
{
  Serial.print("Executing the previously set scenario in ... ");

  for (int cnt_wait = 5; cnt_wait > 0; cnt_wait--)
  {
    if (abortMovement == false)
    {
      Serial.printf("%d ... ", cnt_wait);
      delay(1000);
    }
  }
  Serial.println();
  Serial.println("And here ... we ... go");
  
  int cnt_scenarioMovements = 0;
  while ( (cnt_scenarioMovements < nbr_movementsScenario) && (abortMovement == false) && (digitalRead(PIN_TOGGLE_MODE) == MODE_SCENARIO) )
  {
    stepper.move(scenarioSteps[cnt_scenarioMovements]);
    stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * scenarioSpeed[cnt_scenarioMovements]); // Order matters!!!! 1->speed 2->steps
    
    while ( (stepper.distanceToGo() != 0) && (abortMovement == false) && (digitalRead(PIN_TOGGLE_MODE) == MODE_SCENARIO) )
    {
      stepper.runSpeed();
    }
    cnt_scenarioMovements++;
  }
  
  
  executingScenario = false; // reset the boolean
  Serial.println("Scenario done");

  delay(5000); //waiting before centering the table
  
  Serial.println("Let's check if the table is centered");
  if( (stepper.currentPosition () != 0) && (abortMovement == false) )
  {
    Serial.println("Oups, looks like we didn't stop on the center of the rail, centering now");
    moveTableToCenter(); // You need to have enabled the stepper BEFORE
  }

}// END OF THE FUNCTION

// END OF THE FILE
