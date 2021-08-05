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
#include "SerialRPiComm.h"


// -------------------------- Defines --------------------------

// General
//---------
#define SERIAL_VERBOSE                        // Uncomment to see more debug messages
#define WAIT_FOR_SERIAL                       // Uncomment to wait for the serial port to be opened from the PC end before starting
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console (USB-C of the XIAO)
//#define SHOW_TRIMPOT_VALUE                  // Uncomment to see the calculated frequency and amplitude values in the ISR
#define SERIAL_SHOW_RPI_DATA                  // Uncomment to see what the raspberry pi has sent

// Mode selection
//---------------
const uint8_t PIN_TOGGLE_MODE           = 8;
const uint16_t MS_DEBOUNCE_TIME         = 50; // Mode switch button debouncing time in [ms]

// Modes
//------
#define MODE_SCENARIO   0u                    // Wait orders and scenarios from the Raspberry pi through raspberry pi 30 pin connector (HW UART on the XIAO)
#define MODE_MANUAL     1u                    // Use the 2 trimpots to execute the motion

// Modes
//------
const int TIME_TO_WAIT_IDLE_MS          = 250;  // Time to wait in the switch case when there is no tasks in [ms]


// -------------------------- Global variables ----------------

// Mode toggle switch
//-------------------
volatile bool           flagMode            = false;
volatile unsigned long  last_interrupt_time = 0;

// -------------------------- Functions declaration [16] --------------------------
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
void      flushReceiveAndTransmit (void);
void      readRPiBuffer           (void);
void      parseRPiData            (void);
void      calibrationError        (uint8_t errorCode);






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
        stepper.stop();
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
        stepper.stop();
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
/*-------------------- Functions [10] -------------------*/
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
    // In that case we don't need to be fast: do as many check as you want
		if ( (digitalRead(PIN_LIMIT_RIGHT) == false) && (digitalRead(PIN_LIMIT_LEFT) == false) && (abortMovement == false) )
		{
			#ifdef SERIAL_VERBOSE
			Serial.println("ENABLING the stepper, be careful...");
			#endif
			digitalWrite(PIN_MOTOR_ENA, LOW); // Inverse logic (active low)
			//    digitalWrite(PIN_MOTOR_ENA,HIGH); // Normal logic (active high)
		}
	}
	else // in that case we want to be BLAZING FAST -> digitalWrite is BEFORE the Serial and TODO: used DMA instead
	{
		digitalWrite(PIN_MOTOR_ENA, HIGH); // Inverse logic (active low)
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
    Serial.printf("Moving table back to the center position: %ld [mm], distance to go is %ld [mm] ... ", (long)(0), (long)(stepper.currentPosition() / ustepsPerMM_calib));
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
  *
  */

  #ifdef SERIAL_VERBOSE
    Serial.println("Function calibrateStepper called. For this to work, the stepper must be ALREADY enabled");
  #endif

  if ( (needCalibration == true) && (abortMovement == false) ) // <DEBUG> Why no check for "executingCalib"
  {
    executingCalib  = true; // set the boolean state // <DEBUG> might be redundant
    flagLS_l        = false;
    flagLS_r        = false;

    // Calculate the distance the table is going to move
    totalPossibleTravel_MM = distanceBetweenLS_MM - tableLength_MM;

    // Reset the variable
    totalPossibleTravel_uSteps = 0;

    #ifdef SERIAL_VERBOSE
      Serial.println("Starting calibration now");
    #endif


    //* If the table is touching both (2) LSs, ABORT, beacause that should not happen
    if ( not( (digitalRead(PIN_LIMIT_RIGHT) == 1) && (digitalRead(PIN_LIMIT_LEFT) == 1) ) )
    {
      Serial.println("At least 1 of the LS is NOT ON");

      //* If the table is touching ONLY 1 LS, then move the other direction
      while((digitalRead(PIN_LIMIT_RIGHT) == 1) || (digitalRead(PIN_LIMIT_LEFT) == 1)) // <TODO> add a security to avoid getting stuck, ie if the motor or the LS are not working
      {
        Serial.println("At least 1 of the 2 LS is ON, detriggering now");

        flagLS_l        = false;
        flagLS_r        = false;

        // Let's move to the LEFT (-) if RIGHT is triggered: RELATIVE
        stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
        stepper.move( (long)((((digitalRead(PIN_LIMIT_LEFT) == 1)) ? +1.0 : -1.0) * detriggeringExplorationMicroSteps) ); // This RELATIVE!
        stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal);
        // printStepperState();
        while ( (stepper.distanceToGo() != 0) && (abortMovement == false) )
        {
          stepper.runSpeed();
        }

        Serial.println("De-triggereing movement done. Was that enough though?"); // <DEBUG>
      }

      if (abortMovement == false)
      {

        unsigned long startedWaiting = 0;

        //------------------------------------------------------------------------------------------------------------------------
        //* Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT
        #ifdef SERIAL_VERBOSE
          Serial.println("Trying to reach the 1st LS: the RIGHT one, waiting for an ISR trigger");
        #endif

        flagLS_l        = false;
        flagLS_r        = false;

        // Let's move to the RIGHT (+): RELATIVE
        //--------------------------------------
        stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
        stepper.move( (long)(+1 * calibrationExplorationMicroSteps) ); // This RELATIVE!
        stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal);
        // printStepperState();
        startedWaiting = millis(); // for timeout
        Serial.println("Started the run");
        // We wait here for the ISR associated with the 
        while ( (stepper.distanceToGo() != 0) && (abortMovement == false)  && (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS) )
        {
          stepper.runSpeed();
        }
        #ifdef SERIAL_VERBOSE
          Serial.println("Run ended, why?");
        #endif

        if( (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS) )
        {
          #ifdef SERIAL_VERBOSE
          Serial.println("No timeout for right LS search, that's good");
          #endif

          // <TODO> check if we have an error due to the stepper reaching target w/o triggering a LS --> stepper.distanceToGo() != 0)

          if ( not( (flagLS_r==true) && (flagLS_l==true) ) ) // check that only 1 flag is set
          {
            if(flagLS_r)
            {
              #ifdef SERIAL_VERBOSE
              Serial.println("The correct LS (the RIGHT) has been triggered, that's good");
              #endif
            }
            else
            {
              calibrationError(4);
            }
            // <TODO> check if the other (wrong) ISR flag is set
          }
          else // 2 ISR flag set = 2 LS triggered, should never happen
          {
            calibrationError(3);
          }

        }
        else
        {
          calibrationError(5);
        }

        // Set the right-most position TEMPORARY as 0 to make the step counting easy as
        stepper.setCurrentPosition(0);



        //------------------------------------------------------------------------------------------------------------------------
        //Move left until the RIGHT LS is triggerd (or timeout), if the LEFT one triggers, ABORT
        #ifdef SERIAL_VERBOSE
          Serial.println("Trying to reach the 2nd LS: the LEFT one, waiting for an ISR trigger");
        #endif

        flagLS_l        = false;
        flagLS_r        = false;

        // Let's move to the LEFT (-): RELATIVE
        //--------------------------------------
        stepper.setMaxSpeed(calibrationSpeedMicroStepsPerSeconds_max);
        stepper.move( (long)(-1 * calibrationExplorationMicroSteps) ); // This RELATIVE!
        stepper.setSpeed(((stepper.distanceToGo() > 0) ? +1.0 : -1.0) * calibrationSpeedMicroStepsPerSeconds_normal);
        // printStepperState();
        startedWaiting = millis(); // for timeout
        Serial.println("Started the run");
        // We wait here for the ISR associated with the 
        while ( (stepper.distanceToGo() != 0) && (abortMovement == false)  && (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_L_TIMEOUT_MS) )
        {
          stepper.runSpeed();
        }
        #ifdef SERIAL_VERBOSE
          Serial.println("Run ended, why?");
        #endif

        if( (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_L_TIMEOUT_MS) )
        {
          #ifdef SERIAL_VERBOSE
          Serial.println("No timeout for left LS search, that's good");
          #endif

          // <TODO> check if we have an error due to the stepper reaching target w/o triggering a LS --> stepper.distanceToGo() != 0)

          if ( not( (flagLS_r==true) && (flagLS_l==true) ) ) // check that only 1 flag is set
          {
            if(flagLS_l)
            {
              #ifdef SERIAL_VERBOSE
              Serial.println("The correct LS (the LEFT) has been triggered, that's good");
              #endif
            }
            else
            {
              calibrationError(7);
            }
            // <TODO> check if the other (wrong) ISR flag is set
          }
          else // 2 ISR flag set = 2 LS triggered, should never happen
          {
            calibrationError(6);
          }

        }
        else
        {
          calibrationError(8);
        }




        #ifdef SERIAL_VERBOSE
        Serial.println("All movements done");
        #endif


        // Save the current position as the total distance the table travelled
        totalPossibleTravel_uSteps = (long)( ((stepper.currentPosition() > 0) ? +1 : -1) * stepper.currentPosition() );

        // Conversion factor calculation
        //------------------------------
        if(totalPossibleTravel_MM < 1.0)
        {
          if(totalPossibleTravel_uSteps < 1)
          {
            ustepsPerMM_calib = ( (float)( totalPossibleTravel_uSteps ) ) / totalPossibleTravel_MM;
          }
          else
          {
            calibrationError(10);
          }
        }
        else
        {
          calibrationError(9);
        }
        


      }
      else // abort due to De-triggereing
      {
        calibrationError(2);
      }
    }
    else //* If the table is touching both (2) LSs, ABORT, beacause that should not happen
    {      
      calibrationError(1);
    }
  }  
  else //(needCalibration == true) && (abortMovement == false)
  {
    calibrationError(0);
    // <DEBUG> do not change the state of "calibrationSuccess"
  }



  // End of the function boolean states
  //-----------------------------------

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


//******************************************************************************************
void flushReceiveAndTransmit(void)
{

  /* Since the flush() method only
   clears the transmit buffer, how do 
   you empty the receive (or 
   incoming) buffer? --> like this:
  */

  // Tx
  while(Serial1.available())
  Serial1.read();

  // Rx
  Serial1.flush();


}// END OF THE FUNCTION




//******************************************************************************************
void readRPiBuffer(void)
{
  // const uint16_t  max_nbr_depiledChar = 500; // scope controlled  + cannot be reassigned
  uint16_t  cnt_savedMessage = 0;

  if (Serial1.available())
  {

    Serial1.println("&"); // <DEBUG> send a "I am alive!" to the RPi

    
    if(Serial1.find(RPI_SOM))// test the received buffer for SOM_CHAR_SR. This indicates a start of message from the RPi
    {
      
      /* 
      * Read the HW serial buffer (from the RPi) and depile all the characters.
      * Later we should have a function that parse and understand the commands
      */

      #ifdef SERIAL_SHOW_RPI_DATA
        // <DEBUG> add something
      #endif


      cnt_savedMessage              = 0;
      RPImessage[cnt_savedMessage]  = '$'; // <TODO> Check if we only need 1 or 2 : needed for parsing function
      cnt_savedMessage ++;

      unsigned long startedWaiting = millis();

      while((RPImessage[cnt_savedMessage-1] != ']') && (millis() - startedWaiting <= RPI_DEPILE_TIMEOUT) && (cnt_savedMessage < RX_BUFFER_SIZE)) // <TODO> use "RPI_EOM"
      {
        if (Serial1.available())
        {
          RPImessage[cnt_savedMessage] = Serial1.read();
          cnt_savedMessage++;
        }
      }

      if ((RPImessage[cnt_savedMessage-1] == ']')) // if any EOM found then we parse
      {
        delay(1);             // YES, it IS ABSOLUTELY necessay, do NOT remove it
        parseRPiData();       // <DEBUG> This function is actually empty, this is a placeholder
      }
    }
    memset(RPImessage, 0, RX_BUFFER_SIZE); // clean the message field anyway
  }
}// END OF THE FUNCTION


//******************************************************************************************
void parseRPiData(void)
{
  // <Placeholder> <Use the code for the RS1D sesimometer: void parseGeophoneData(void)>

}// END OF THE FUNCTION

//******************************************************************************************
void calibrationError(uint8_t errorCode)
{
  #ifdef SERIAL_VERBOSE
  Serial.println(" /!\\ Calibration error /!\\");
  Serial.printf("Error code: %d \r\n", errorCode);
  Serial.print("Explanation: ");
  #endif

  switch(errorCode) 
  {
      case 0 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Calibration is called but is currently unnecessary");
        #endif
        break;
      }
      case 2 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during de-triggereing");
        #endif
        break;
      }
      case 3 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during RIGHT search: 2 ISR flag set = 2 LS triggered, should never happen");
        #endif
        break;
      }
      case 4 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during RIGHT LS search: right ISR flag was NOT set");
        #endif
        break;
      }
      case 5 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during RIGHT LS search: Timeout");
        #endif
        break;
      }
      case 6 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during LEFT search: 2 ISR flag set = 2 LS triggered, should never happen");
        #endif
        break;
      }
      case 7 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during LEFT LS search: left ISR flag was NOT set");
        #endif
        break;
      }
      case 8 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during LEFT LS search: Timeout");
        #endif
        break;
      }
      case 9 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during calibration coefficient calculation: totalPossibleTravel_MM too small");
        #endif
        break;
      }
      case 10 :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Error during calibration coefficient calculation: totalPossibleTravel_uSteps too small");
        #endif
        break;
      }
      default :
      {
        #ifdef SERIAL_VERBOSE
        Serial.println("Unknown error code");
        #endif
        break;
      }
   }


  #ifdef SERIAL_VERBOSE
  Serial.print("Setting boolean states ... ");
  #endif

  calibrationSuccess  = false;
  abortMovement       = true;

  #ifdef SERIAL_VERBOSE
  Serial.println("done");
  #endif

}// END OF THE FUNCTION



// END OF THE FILE
