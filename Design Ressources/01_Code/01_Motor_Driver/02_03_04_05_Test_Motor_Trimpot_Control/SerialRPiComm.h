/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-26
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

const unsigned long RPI_DEPILE_TIMEOUT   = 100;  // in [ms]
#define   RPI_TIMEOUT           100     // For the character search in buffer
#define   COMMAND_BAUD_RATE     921600  // Baudrate in [bauds] for serial communication to the Raspberry Pi (RPi GPIO 15 for XIAO TX-6/RPi GPIO 14 for XIAO TX-7)
#define   RX_BUFFER_SIZE        20      //10 actually needed, margin of 10 observed

// -------------------------- Global variables ----------------

char  RPImessage[RX_BUFFER_SIZE];            // Char array containing the message comming back from the raspberry pi through the HW UART connection
bool  goodMessageReceived_flag    = false;   // Set to "bad message" first


// END OF THE FILE