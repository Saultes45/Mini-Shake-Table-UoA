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

// Limit switches pins
//--------------------
const uint8_t PIN_LIMIT_RIGHT     = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT      = 10; // Digital input 3.3V
const uint16_t LS_DEBOUNCE_TIME   = 250;    //100 // millisecond button debouncing time



// -------------------------- Global variables ----------------

// Limit switches
//---------------
volatile bool flagLS_r  = false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;

volatile bool flagLS_l  = false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;




// END OF THE FILE
