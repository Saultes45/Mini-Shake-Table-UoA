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

// -------------------------- Defines --------------------------

// Timeouts
//----------
const unsigned long STEPPER_CALIB_REACH_LS_R_TIMEOUT_MS         = 8000;    // in [ms]
const unsigned long STEPPER_CALIB_REACH_LS_L_TIMEOUT_MS         = 12000;    // in [ms]

// -------------------------- Global variables ----------------

// Stepper calibration
//---------------------

float   distanceBetweenLS_MM          = 580.0;      // For stepper calibration, might be changed by RPI commands
float   tableLength_MM                = 400.0;      // For stepper calibration, might be changed by RPI commands
float   totalPossibleTravel_MM        = 0.0;      // For stepper calibration, is calculated during calibration by equation
long    totalPossibleTravel_uSteps    = 0;          // For stepper calibration, is changed during calibration
float   ustepsPerMM_calib             = 0.0;        // This parameter holds the calibration parameter



// Boolean states
//---------------
bool    needCalibration           = true;       // Indicates if there is a current good distance calibration done
bool    executingCalib            = false;      // Indicates if we are currently in distance calibration with the Limit Switches
bool    calibrationSuccess        = false;      // A variable that tells if the calibration was sucessful

// Calibration error code
//------------------------


// END OF THE FILE
