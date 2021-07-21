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
#define USE_TC3

// Trimpots sequencing
#ifdef USE_TC3
#include <TimerTC3.h>
#else
#include <TimerTCC0.h>
#endif

// Trimpots filtering
#include "MedianFilterLib2.h"

// Stepper motor
#include <AccelStepper.h>

// -------------------------- Defines --------------------------

// General
#define SERIAL_VERBOSE
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

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
 * is currently on the motor driver 
 */
const uint8_t indx_microSteppingFactorList              = 0; 
const float   max_allowedMicroStepsPerSeconds           = 10000.0;//500.0 * 200.0; //10000.0; //10000000
const float   max_allowedMicroStepsPerSecondsPerSeconds = 14000.0;//200.0 * 200.0;//750.0; // //750
const long    nativePulsesPerRevolution                 = ((long)200); // This parameter is from the motor and CANNOT be changed
const long    microstepsPerRevolution                   = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;
const float   centeringSpeedMicroStepsPerSecondsPerSeconds = 700.0;
const float   centeringSpeedMicroStepsPerSeconds           = 500.0;



// Limit switches pins
//--------------------
const uint8_t PIN_LIMIT_RIGHT     = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT      = 10; // Digital input 3.3V
const uint16_t LS_DEBOUNCE_TIME   = 250;    //100 // millisecond button debouncing time

// Trimpot pins
//--------------
const uint8_t PIN_TRMPT_FREQ            = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL            = A1; // Analog input 3.3V
const uint32_t FREQUENCY_READ_TRIMPOTS  = 10000; // in [Hz] 1000
const uint16_t NBR_SAMPLES_MEDIAN       = 50;
const float XIAO_ADC_MAX_LSB            = 1023.0; // <<--- This MUST be a float!!!!!
// Trimpots deadbands ([LSB] or [V] or [frequency]) -> LSB
const uint16_t TRMPT_DEADBAND_FREQ      = 25;
const uint16_t TRMPT_DEADBAND_AMPL      = 25;

// Mode selection
//---------------
const uint8_t PIN_TOGGLE_MODE           = 8;    // Can be any I/O pin from 0 to 10
const uint16_t MS_DEBOUNCE_TIME         = 50;      // millisecond button debouncing time

// Modes
#define MODE_AUTO   0u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
#define MODE_MANUAL 1u  // Use the 2 trimpots to execute the motion



// -------------------------- Global variables ----------------


// Limit switches
//---------------
volatile bool flagLS_r  = false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;

volatile bool flagLS_l  = false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;

// Mode toggle switch
//-------------------
volatile bool           flagMode            = false;
volatile unsigned long  last_interrupt_time = 0;

// Trimpots
//---------

// Trimpot: Frequency
uint16_t median1          = 0;
uint16_t sensorValue1     = 0;
MedianFilter2<int> medianFilter1(NBR_SAMPLES_MEDIAN);
float current_trimpotFrequency_filtered = 0.0;

// Trimpot: Amplitude
uint16_t median2        = 0;
uint16_t sensorValue2   = 0;
MedianFilter2<int> medianFilter2(NBR_SAMPLES_MEDIAN);
float current_trimpotAmplitude_filtered = 0.0;

// precision  (trimpot_frequency_max - trimpot_frequency_min) / 1024
// 0.97Hz/LSB
float trimpot_frequency_range_max_lsb   = XIAO_ADC_MAX_LSB; // in [LSB]
float trimpot_frequency_range_min_lsb   = 6.0;        // in [LSB]
float trimpot_frequency_max             = 1000;       // in [Hz]
float trimpot_frequency_min             = 0.0;        // in [Hz]
// to use with stepper.setAcceleration

// amplitude  (trimpot_amplitude_max - trimpot_amplitude_min) / 1024
// 9.765625E-4 stepper shaft rotations/LSB
float trimpot_amplitude_range_max_lsb   = XIAO_ADC_MAX_LSB;  // in [LSB]
float trimpot_amplitude_range_min_lsb   = 6.0;         // in [LSB]
float trimpot_amplitude_max             = 0.5;         // in [stepper shaft rotations]
float trimpot_amplitude_min             = 0.0;         // in [stepper shaft rotations]
// to use with : stepper.moveTo


// Stepper motor driver 
//----------------------
/* Define a stepper and the pins it will use, and precise that
 *  we use an external dedicated stepper driver, thus we have 2 pins: 
 *  STEP and DIR (ENA is handled manually) 
 */
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);


// Stepper calibration
//---------------------
float ustepsPerMM_calib       = 14;        // <DEBUG> This parameter holds the calibration parameter
float distanceBetweenLS_MM    = 500.0;     // For stepper calibration
long distanceBetweenLS_uSteps = 0;          // For stepper calibration
//bool abortCalibration         = true;       // A variable that tells if we need to stop the calibration
bool calibrationSuccess       = false;      // A variable that tells if the calibration was sucessful

bool needCalibration          = true;       // Indicates if there is a current good distance calibration done
bool abortMovement         = true;       // A (unified) variable that tells if we need to stop the centering/calibration/scenario

// Local Scenario (not needed here but for compatibility)
//----------------
      //bool  abortScenario                         = false; // a variable that store potential reason to stop a scenario



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
      enableStepper (false);
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
      enableStepper (false);
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

  // Raw values
  //  Serial.print(sensorValue1);
  //  Serial.print(" ");
  //  Serial.print(median1);
  //  Serial.print(" ");
  //  Serial.print(sensorValue2);
  //  Serial.print(" ");
  //  Serial.println(median2);

  // Prints

  Serial.print  (current_trimpotFrequency_filtered);
  Serial.print(" ");
  Serial.println(current_trimpotAmplitude_filtered);

}

}

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


  // Checking the flag of all the ISRs [3]
  //--------------------------------------
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


  // if ther are no errors then we can continue
  if (abortMovement == false)
  {
    
  }

}


/*------------------------------------------------------*/
/*-------------------- Functions [?] -------------------*/
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
      Serial.print("ENABLING the stepper, be careful...");
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
  calibrationSuccess       = true;      // A variable that tells if the calibration was sucessful
  

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
