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


// -------------------------- Defines --------------------------
// General
#define SERIAL_VERBOSE                        // uncomment to see more debug messages 
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console
//#define SHOW_TRIMPOT_VALUE                  // uncomment to see the calculated frequency and amplitude values in the ISR


// Mode selection
//---------------
const uint8_t PIN_TOGGLE_MODE           = 8;    // Can be any I/O pin from 0 to 10
const uint16_t MS_DEBOUNCE_TIME         = 50;      // millisecond button debouncing time

// Modes
#define MODE_AUTO   0u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
#define MODE_MANUAL 1u  // Use the 2 trimpots to execute the motion


// -------------------------- Global variables ----------------

// Mode toggle switch
//-------------------
volatile bool           flagMode            = false;
volatile unsigned long  last_interrupt_time = 0;

// -------------------------- Functions declaration [7] --------------------------
void      pinSetUp            (void);
void      attachISRs          (void);
void      enableTrimpots      (bool enableOrder);
void      enableStepper       (bool enableOrder);
void      displayStepperSettings(void);
void      moveTableToCenter   (void);
void      calibrateStepper    (void);




// -------------------------- ISR [4] ----------------

//******************************************************************************************
void LW_r_ISR()
{
  noInterrupts();
  unsigned long interrupt_time = millis();
  // If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time_ls_r > LS_DEBOUNCE_TIME)
  {
    flagLS_r = true;
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_RIGHT));
    if(digitalRead(PIN_LIMIT_RIGHT))
    {
      enableStepper (executingCalib); // disable the stepper only if NOT in calibration
      enableTrimpots(false);
      abortMovement = true;
      stateLS_r = true;
      
    }
    else
    {
      //enableStepper(false);
      //enableTrimpots(true); // enable back only if in Manual
      stateLS_r = false;
    }
   last_interrupt_time_ls_r = interrupt_time;
  }
  
  interrupts();
}

//******************************************************************************************
void LW_l_ISR()
{
  noInterrupts();
  unsigned long interrupt_time = millis();
  // If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time_ls_l > LS_DEBOUNCE_TIME)
  {
    flagLS_l = true;
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_LEFT));
    if(digitalRead(PIN_LIMIT_LEFT))
    {
      enableStepper (executingCalib); // disable the stepper only if NOT in calibration
      enableTrimpots(false);
      abortMovement = true;
      stateLS_l = true;
    }
    else
    {
      //enableStepper(false);
      stateLS_l = false;
      //enableTrimpots(true); // enable back only if in Manual <-- This triggers and a timer interrupt, MUST be LAST
      
    }
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
    flagMode = true;

    if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
    {
      // If we are in Manual, then attach the timer interrupt to read the trimpots
      enableTrimpots(true);
    }
    else
    {
      // If we are in Auto or (Scenario), then detach the
      //  timer interrupt to stop reading the trimpots
      enableTrimpots(false);
    }
    
  }
  last_interrupt_time = interrupt_time;
  //interrupts();
}

//******************************************************************************************
void timerTrimpotISR()
{

  /* NOTICE:
*  Do NOT put noInterupts(); here because this is a low priority and long execution time "task"
*  Limit switches IRS should be able to trigger in there
*/

// 1st check if we are in manual mode because scenario mode doesn't use 
if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
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
  //  Serial.print(sensorValue1);
  //  Serial.print(" ");
  //  Serial.print(median1);
  //  Serial.print(" ");
  //  Serial.print(sensorValue2);
  //  Serial.print(" ");
  //  Serial.println(median2);
  // Final values
  Serial.print  (current_trimpotFrequency_filtered);
  Serial.print(" ");
  Serial.println(current_trimpotAmplitude_filtered);
  #endif

}

}


/*------------------------------------------------------*/
/*-------------------- Functions [7] -------------------*/
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
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LW_r_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LW_l_ISR, CHANGE);

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
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second
  Serial.print("Setting max speed to: ");
  Serial.println(max_allowedMicroStepsPerSeconds); // Returns the most recent speed in [steps per second]
  stepper.setMaxSpeed(max_allowedMicroStepsPerSeconds);
  Serial.print("New max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second (float)

  Serial.print("Setting acceleration to ");
  Serial.println(max_allowedMicroStepsPerSecondsPerSeconds);
  stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds);
  // No function to read back set acceleration
  // NOTICE: Calling setAcceleration() is expensive, since it requires a square root to be calculated.

}// END OF THE FUNCTION

//******************************************************************************************
void  moveTableToCenter   (void)
{

  #ifdef SERIAL_VERBOSE
  Serial.println("Function moveTableToCenter called, for this to work, you need to have the stepper enabled ALREADY");
  Serial.printf("Moving table back to the center position: %ld [mm] ... ", (long)(distanceBetweenLS_MM/2.0 * ustepsPerMM_calib));
  #endif

  stepper.setSpeed(centeringSpeedMicroStepsPerSeconds); // Order matters!!!! 1-> max speed (needed for accel) 2->accel 3->target pos (steps)
  stepper.setAcceleration(centeringSpeedMicroStepsPerSecondsPerSeconds);
  stepper.moveTo( (long)(distanceBetweenLS_MM/2.0 * ustepsPerMM_calib) );
  while ((stepper.distanceToGo() != 0) && (abortMovement == false) )
  {
    stepper.run();
  }

  /* Parameters:  position (long) The position in steps of wherever the 
   *  motor happens to be right now 
  *  Resets the current position of the motor, so that 
  *  wherever the motor happens to be right now is considered 
  *  to be the new 0 position. Useful for setting a zero position 
  *  on a stepper after an initial hardware positioning move. Has 
  *  the side effect of setting the current motor speed to 0.
  */

   stepper.setCurrentPosition(0);

  #ifdef SERIAL_VERBOSE
  Serial.println("done");
  #endif

}// END OF THE FUNCTION

//******************************************************************************************
void  calibrateStepper (void) // <TODO> This is currently a placeholder, use the code from "05_02_TestCalibartion_StepperDistance_Limitswitches"
{

  #ifdef SERIAL_VERBOSE
  Serial.println("Function calibrateStepper called, for this to work, you need to have the stepper enabled ALREADY");
  Serial.print("Starting calibration...");
  #endif

  // (Re)setting some variables 
  // <DEBUG>
  // This is the START position, get the table (motor) position and save it 
    distanceBetweenLS_uSteps = 500; //stepper.currentPosition(); // This is also a distance since we marked the 0 as the right-most 
    
    //* Calculate how many steps have been executed to travel the USER-DEFINED distance between the 2 LS

    ustepsPerMM_calib = distanceBetweenLS_uSteps / distanceBetweenLS_MM;

    Serial.println("Calibration success!");
    Serial.printf("The calibrated step distance bewteen the 2 limit switches is %ld \r\n", distanceBetweenLS_uSteps);
    Serial.printf("The user-defined distance bewteen the 2 limit switches is %f [mm] \r\n", distanceBetweenLS_MM);
    Serial.printf("The calibration ratio is: %f [mm/steps]\r\n", ustepsPerMM_calib);
    
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



}// END OF THE FUNCTION


// END OF THE FILE
