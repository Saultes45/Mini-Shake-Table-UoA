/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-23
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

// -------------------------- Defines --------------------------
// Boolean states
bool    executingScenario           = false;      // Indicates if we are currently in distance calibration with the Limit Switches

// Local Scenario
//----------------
const int   nbr_movementsScenario           = 10; // This variable is const because it defuines the size of the memory we can allocate to scenario parameters
long  scenarioSteps[nbr_movementsScenario]  = {0};
float scenarioSpeed[nbr_movementsScenario]  = {0.0};
// bool  abortScenario                         = true; // a variable that store potential reason to stop a scenario
