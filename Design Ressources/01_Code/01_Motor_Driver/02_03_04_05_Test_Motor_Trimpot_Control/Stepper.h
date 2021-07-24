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

// Stepper motor
#include <AccelStepper.h>

// -------------------------- Defines --------------------------


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
const long    nativePulsesPerRevolution                 = ((long)200); // This parameter is from the motor and CANNOT be changed 
const long    microstepsPerRevolution                   = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;

const uint    singleCycleRepetition                     = 1; // In manual mode: the number of repetition of the same pattern without changing the parameters

//Parameter for the different modes [4]
//-------------------------------------

//  Overall max
const float   max_allowedMicroStepsPerSeconds             = 84000;
const float   max_allowedMicroStepsPerSecondsPerSeconds   = 84000.0 * 3.5; // max speed acheved in 1/3.5 = 286 [ms]


// Mode 1 - Homing/Calibration/Detriggering - no acceleration, slow speed
//const float   calibrationSpeedMicroStepsPerSecondsPerSeconds = 500.0;
const float   calibrationSpeedMicroStepsPerSeconds_max          = 100.0;
const float   calibrationSpeedMicroStepsPerSeconds_normal       = 100.0;
const long    calibrationExplorationMicroSteps                  = 10;

// Mode 2 - Centering - acceleration, slow speed
//const float   centeringSpeedMicroStepsPerSecondsPerSeconds = 500.0;
const float   centeringSpeedMicroStepsPerSeconds_max          = 500.0;
const float   centeringSpeedMicroStepsPerSeconds_normal       = 500.0;
const float   centeringSpeedMicroStepsPerSecondsPerSeconds = max_allowedMicroStepsPerSecondsPerSeconds;

// Mode 3 - Single cycle movement - speed over acceleration, we DO need acceleration
const float   manualSpeedMicroStepsPerSecondsPerSeconds = max_allowedMicroStepsPerSecondsPerSeconds;
const float   manualSpeedMicroStepsPerSeconds             = max_allowedMicroStepsPerSeconds;

// Mode 4 - Scenario - accel is needed but it must be time accurate
const float   scenarioSpeedMicroStepsPerSecondsPerSeconds = max_allowedMicroStepsPerSecondsPerSeconds;
const float   scenarioSpeedMicroStepsPerSeconds           = max_allowedMicroStepsPerSeconds;




// -------------------------- Global variables ----------------


// Stepper motor driver 
//----------------------
/* Define a stepper and the pins it will use, and precise that
 *  we use an external dedicated stepper driver, thus we have 2 pins: 
 *  STEP and DIR (ENA is handled manually) 
 */
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);

// boolean states
bool abortMovement = false;

// END OF THE FILE
