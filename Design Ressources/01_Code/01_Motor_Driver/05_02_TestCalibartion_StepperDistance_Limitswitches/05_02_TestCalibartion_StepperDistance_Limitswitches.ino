/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   :
* Creation date : 2021-07-19
* Version       : 0.1 (finished on 2021-..-..)
* Modifications :
* Known bugs    :
*
*
* Possible Improvements
*
*
* Notes
*
*
* Ressources (Boards + Libraries Manager)
*
*
* TODO
* findout why the the first movement is always 1/2, I don't know why
* Single Cycle IOBUS: allowing I/O pins to be manipulated in a single instruction cycle
* DMA: https://qiita.com/nanase/items/406a8a848d1b259d3af4#dma
* 
* Theory of operation:
* If the table is touching both (2) LSs, ABORT, that should not happen
* If the table is touching 1 LS, move the other direction
* Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT
* This is the START position. 
* Move to the the LEFT until (timeout) or LEFT LS triggers, if the LFT one triggers, ABORT
* Calculate how many steps have been executed to thatvel the KNOWN distance between the 2 LS
* 
*
* DO WE USE THE isr FLAG FOR THE isr OR THAT: digitalRead(PIN_LIMIT_RIGHT) == 1) 
*
* ========================================
*/

// -------------------------- Includes --------------------------
#include <AccelStepper.h>

// -------------------------- Defines --------------------------

#define SERIAL_VERBOSE
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

// Limit switches pins
//---------------------
const uint8_t PIN_LIMIT_RIGHT     = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT      = 10; // Digital input 3.3V
const uint16_t LS_DEBOUNCE_TIME   = 50;    //100 // millisecond button debouncing time


// Stepper motor driver pins (logic low)
//--------------------------------------
const uint8_t PIN_MOTOR_DIR    = 3; // Digital output 3.3V
const uint8_t PIN_MOTOR_STEP   = 4; // Digital output 3.3V
const uint8_t PIN_MOTOR_ENA    = 5; // Digital output 3.3V, manual logic



// Stepper conversion factors
//-----------------------------
/*
 * Handled exclusively by the motor driver - I know, it 
 * is NOT sorted ascendedly but what can I do?
 */
const long microSteppingFactorList[]                  = {1, 4, 8, 16, 32, 64, 128, 256, 5, 10, 20, 25, 40, 50, 100, 200}; 
/* Says to this program which physical microSteppingFactor 
 *  is currently on the motor driver 
 */
const uint8_t indx_microSteppingFactorList              = 0; 
const long    nativePulsesPerRevolution                 = ((long)200); // This parameter is from the motor and CANNOT be changed
const long    microstepsPerRevolution                   = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;
const float   max_allowedMicroStepsPerSeconds           = 10000.0;
const float   max_allowedMicroStepsPerSecondsPerSeconds = 14000.0;
// Stepper calibration
//---------------------
const float   max_allowedMicroStepsPerSeconds_calib             = 100.0;
const float   max_allowedMicroStepsPerSecondsPerSeconds_calib   = 150.0;
const float   MicroStepsPerSeconds_calib                        = 100;
const long    ustepsToMoveCalib_untrigger                       = 150;
const long    ustepsToMoveCalib_roving                          = 100000; // <-- This must be very large to enable at least a full table travel
const unsigned long STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS         = 8000;    // in [ms]
const unsigned long STEPPER_CALIB_REACH_LS_L_TIMEOUT_MS         = 12000;    // in [ms]


// Local Scenario
//----------------
      bool  abortScenario                         = true; // a variable that store potential reason to stop a scenario


 
// -------------------------- Global variables ----------------

// Limit switches L+R
//--------------------
volatile bool flagLS_r   = false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;
volatile bool flagLS_l   = false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;


// Stepper calibration
//---------------------
float ustepsPerMM_calib       = 0.0;        // This parameter holds the calibration parameter
float distanceBetweenLS_MM    = 1000.0;     // For stepper calibration
long distanceBetweenLS_uSteps = 0;          // For stepper calibration
bool abortCalibration         = true;       // A variable that tells if we need to stop the calibration
bool calibrationSuccess       = false;      // A variable that tells if the calibration was sucessful

/* Define a stepper and the pins it will use, and precise that
 *  we use an external dedicated stepper driver, thus we have 2 pins: 
 *  STEP and DIR (ENA is handled manually) 
 */
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);

// -------------------------- Functions declaration --------------------------
void enableStepper(bool enableOrder);




// -------------------------- ISR [2] ----------------

//******************************************************************************************
void LW_r_ISR()
{
  noInterrupts();
  unsigned long interrupt_time = millis();
  // If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time_ls_r > LS_DEBOUNCE_TIME)
  {
    flagLS_r          = true; // make sure the loop knows an ISR has been triggered
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_RIGHT));
    if(digitalRead(PIN_LIMIT_RIGHT))
    {
      stateLS_r = true;
      //enableTrimpots(false);
       enableStepper(false);     // disable the stepper ASAP, can be enabled back in the loop
       abortScenario     = true; // make sure no other stepper movement are executed
        abortCalibration  = true; // make sure no other stepper movement are executed
    }
    else
    {
      stateLS_r = false;
      //enableTrimpots(true); // enable back only if in Manual
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
    flagLS_l          = true; // make sure the loop knows an ISR has been triggered
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_LEFT));
    if(digitalRead(PIN_LIMIT_LEFT))
    {
      stateLS_l = true;
      //enableTrimpots(false);
      enableStepper(false);     // disable the stepper ASAP, can be anbaled back in the loop
      abortScenario     = true; // make sure no other stepper movement are executed
      abortCalibration  = true; // make sure no other stepper movement are executed
    }
    else
    {
      stateLS_l = false;
      //enableTrimpots(true); // enable back only if in Manual
    }
   last_interrupt_time_ls_l = interrupt_time;
  }
  
  interrupts();
}



// -------------------------- SetUp --------------------------
void setup() 
{

  delay(5000);

  Serial.begin(CONSOLE_BAUD_RATE); //Begin serial communication (USB)
  Serial.println("---------------------------------");
  
  // initialize digital pin PIN_MOTOR_ENA as an output.
  pinMode(PIN_MOTOR_ENA, OUTPUT);

  // Disable the motor immediatly (external toggle switch should also be in disable positon)
  enableStepper(false);

    // Limit switches
    //----------------
   /* Attach each a different ISR
  *  For each we want to know when it is
  *  triggered and released so we can enable the drive again if necesary
  */
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LW_r_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LW_l_ISR, CHANGE);

  // Display settings to UART
  //-------------------------

  Serial.println("---------------------------------");
  
  Serial.print("Current micro-steps settings: ");
  Serial.println(microSteppingFactorList[indx_microSteppingFactorList]);

  Serial.print("Current micro-steps per rotation: ");
  Serial.println(microstepsPerRevolution);

  Serial.println("---------------------------------");

  Serial.print("Current max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second
  Serial.print("Setting max speed to ");
  Serial.println(max_allowedMicroStepsPerSeconds); // Returns the most recent speed in steps per second
  stepper.setMaxSpeed(max_allowedMicroStepsPerSeconds);
  Serial.print("New max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second

  Serial.print("Setting acceleration to ");
  Serial.println(max_allowedMicroStepsPerSecondsPerSeconds);
  stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds);
  // No function to read back set acceleration
  // NOTICE: Calling setAcceleration() is expensive, since it requires a square root to be calculated.

  Serial.println("---------------------------------");
  abortCalibration = false;


}// END OF SETUP



// -------------------------- Loop --------------------------
void loop() 
{
  Serial.println("Loop"); // <DEBUG>
  if (abortCalibration == false)
  {
    /* Theory of operation:
    * Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT
    * This is the START position. 
    * Move to the the LEFT until (timeout) or LEFT LS triggers, if the LFT one triggers, ABORT
    */
    
    //* If the table is touching both (2) LSs, ABORT, beacause that should not happen
    if (not( (digitalRead(PIN_LIMIT_RIGHT) == 1) && (digitalRead(PIN_LIMIT_LEFT) == 1) ))
    {
      Serial.println("At least 1 of the LS is not ON"); // <DEBUG>
      enableStepper(true); // Once the stepper is enabled, it must stay ENABLED until the end of the calibration
      delay(1000); // wait here for the lock or snap in place of the motor
      
      //* If the table is touching ONLY 1 LS, then move the other direction
      if((digitalRead(PIN_LIMIT_RIGHT) == 1) || (digitalRead(PIN_LIMIT_LEFT) == 1))
      {
        Serial.println("At least 1 of the 2 LS is ON, detriggering..."); // <DEBUG>
        // <TODO> take into account the direction
        // The stepper should already be enabled

        Serial.println("I am here");
        stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds_calib);
        stepper.setSpeed(MicroStepsPerSeconds_calib); // Order matters!!!! 1->speed 2->steps
        stepper.moveTo(ustepsToMoveCalib_untrigger);
        while ((stepper.distanceToGo() != 0) && (abortCalibration == false))
        {
          stepper.run();
        }
        Serial.println("Movement done"); // <DEBUG>
        Serial.print("abortCalibration:"); // <DEBUG>
        Serial.println(abortCalibration); // <DEBUG>
        
      }

      //* Wherever the motor is NOW, move right until the RIGHT LS is triggerd (or timeout), if the LFT one triggers, ABORT

      // Trying to reach the 1st LS: the right one
      Serial.println("Trying to reach the 1st LS: the right one"); // <DEBUG>
      stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds_calib);
      stepper.setSpeed(MicroStepsPerSeconds_calib); // Order matters!!!! 1->speed 2->steps
      stepper.moveTo(ustepsToMoveCalib_roving);

      unsigned long startedWaiting = millis();

      Serial.println("Started the run "); // <DEBUG>
      while( (flagLS_l == false) && (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS) && (stepper.distanceToGo() != 0) && (abortCalibration == false))
      {
        stepper.run();
      }
      Serial.println("Run ended, why?"); // <DEBUG>

      
      if (flagLS_l == true)
      {
        Serial.println("Then everything went according to the plan, no timeout"); // <DEBUG>
        // Then everything went according to the plan, no timeout
        
        /* Parameters:  position (long) The position in steps of wherever the 
        *  Resets the current position of the motor, so that 
        *  wherever the motor happens to be right now is considered 
        *  to be the new 0 position. Useful for setting a zero position 
        *  on a stepper after an initial hardware positioning move. Has 
        *  the side effect of setting the current motor speed to 0.
        *  motor happens to be right now 
        */

        /* Parameters:  Returns 
        * the current motor position (long) in steps. Positive is
        * clockwise from the 0 position.
        */

        stepper.setCurrentPosition(0);

        // Trying to reach the 1st LS: the right one
        stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds_calib);
        stepper.setSpeed(MicroStepsPerSeconds_calib); // Order matters!!!! 1->speed 2->steps
        stepper.moveTo(-ustepsToMoveCalib_roving);

        startedWaiting = millis();

        while( (flagLS_r == false) && (millis() - startedWaiting <= STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS) && (stepper.distanceToGo() != 0) && (abortCalibration == false))
        {
          stepper.run();
        }

        if (flagLS_l == true)
        {
          // This is the START position, get the table (motor) position and save it 
          distanceBetweenLS_uSteps = stepper.currentPosition(); // This is also a distance since we marked the 0 as the right-most 
          
          //* Calculate how many steps have been executed to travel the USER-DEFINED distance between the 2 LS

          ustepsPerMM_calib = distanceBetweenLS_uSteps / distanceBetweenLS_MM;

          Serial.println("Calibration success!");
          Serial.printf("The calibrated step distance bewteen the 2 limit switches is \r\n", distanceBetweenLS_uSteps);
          Serial.printf("The user-defined distance bewteen the 2 limit switches is %f [mm] \r\n", distanceBetweenLS_MM);
          Serial.printf("The calibration ratio is: %f [mm/steps]\r\n", ustepsPerMM_calib);
          
          calibrationSuccess = true;
        }
        else // if no timeout trying to reach the 2nd LS: the left one
        {
          abortCalibration = true;
          enableStepper(false);
        }
      }
      else // if no timeout trying to reach the 1st LS: the right one
      {
        abortCalibration = true;
        enableStepper(false);
      } 
    }
    else
    {
      //* If the table is touching both (2) LSs, ABORT, that should not happen
      abortCalibration = true;
      enableStepper(false);
    }
      

    
    if (calibrationSuccess == false)
    {
      Serial.println("Calibration failed. Reset and try again");
    // abortCalibration = true;
    }
  
  
  } //if (abortCalibration == false)
  
  delay(1000);

}// END OF LOOP



/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/
//******************************************************************************************
void enableStepper(bool enableOrder)
{
  if (enableOrder)
  {
    if (digitalRead(PIN_LIMIT_RIGHT) == false & digitalRead(PIN_LIMIT_LEFT) == false && abortCalibration == false)
    {
      Serial.print("ENABLING the stepper, be careful...");
      digitalWrite(PIN_MOTOR_ENA,LOW); // Inverse logic (active low)
//    digitalWrite(PIN_MOTOR_ENA,HIGH); // Normal logic (active high)
    }
  }
  else
  {
    Serial.print("DISABLING the stepper...");
    digitalWrite(PIN_MOTOR_ENA,HIGH); // Inverse logic (active low)
//    digitalWrite(PIN_MOTOR_ENA,LOW); // Normal logic (active high)
  }
  Serial.println("done");
  
}


// END OF FILE
