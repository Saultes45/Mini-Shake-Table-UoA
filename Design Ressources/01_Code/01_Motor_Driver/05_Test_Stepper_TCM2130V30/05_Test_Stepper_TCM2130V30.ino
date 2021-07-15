/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   :
* Creation date : 2021-07-16
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
*
* ========================================
*/

// -------------------------- Includes --------------------------
#include <AccelStepper.h>

// -------------------------- Defines --------------------------

#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console


// Stepper motor driver pins (logic low)
const uint8_t PIN_MOTOR_STEP   = 5; // Digital output 3.3V
const uint8_t PIN_MOTOR_DIR    = 4; // Digital output 3.3V
const uint8_t PIN_MOTOR_ENA    = 6; // Digital output 3.3V, manual logic

const uint16_t microSteppingFactorList[]    = {1, 4, 8, 16, 32, 64, 128, 256, 5, 10, 20, 25, 40, 50, 100, 200}; // Handled exclusively by the motor driver - I know, it is not sorted ascendedly but what can I do?
const uint8_t indx_microSteppingFactorList  = 0; // Says to this program which physical microSteppingFactor is currently on the motor driver
const uint8_t nativePulsesPerRevolution     = 200; // This parameter is from the motor and CANNOT be changed
const uint8_t microstepsPerRevolution       = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;

uint16_t nbr_desiredRevolutions = 1;

// -------------------------- Global variables ----------------


// -------------------------- SetUp --------------------------
void setup() 
{

  Serial.println("---------------------------------");
  
    // initialize digital pin PIN_MOTOR_ENA as an output.
  pinMode(PIN_MOTOR_ENA, OUTPUT);

  // Disable the motor (logic inverse)
  digitalWrite(PIN_MOTOR_ENA,HIGH);
  

}



// -------------------------- Loop --------------------------
void loop() 
{
  

}
