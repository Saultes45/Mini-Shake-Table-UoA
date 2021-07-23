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


// -------------------------- Global variables ----------------
// Stepper calibration
//---------------------

float   distanceBetweenLS_MM      = 500.0;      // For stepper calibration
long    distanceBetweenLS_uSteps  = 0;          // For stepper calibration
float   ustepsPerMM_calib         = 0.0;        // This parameter holds the calibration parameter

// Boolean states
bool    needCalibration           = true;       // Indicates if there is a current good distance calibration done
bool    executingCalib            = false;      // Indicates if we are currently in distance calibration with the Limit Switches
bool    calibrationSuccess        = false;      // A variable that tells if the calibration was sucessful




// END OF THE FILE
