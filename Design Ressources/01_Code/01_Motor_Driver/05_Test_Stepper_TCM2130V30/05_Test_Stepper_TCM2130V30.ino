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
const uint16_t microSteppingFactorList[]    = {1, 4, 8, 16, 32, 64, 128, 256, 5, 10, 20, 25, 40, 50, 100, 200}; 
/* Says to this program which physical microSteppingFactor 
 *  is currently on the motor driver 
 */
const uint8_t indx_microSteppingFactorList  = 0; 
const uint8_t nativePulsesPerRevolution     = 200; // This parameter is from the motor and CANNOT be changed
const uint8_t microstepsPerRevolution       = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;



// -------------------------- Global variables ----------------


// Define a stepper and the pins it will use
// Precise that we use a dedicated stepper driver, thus we have 2 pins: STEP and DIR (ENA is handled manually)
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);


// Stepper motor movement planning
//--------------------------------
uint16_t nbr_desiredRevolutions = 1;

// -------------------------- Functions declaration --------------------------
void enableStepper(bool enableOrder);


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

  

  // Display settings to UART
  //-------------------------
  Serial.print("Current micro-steps settings: ");
  Serial.println(microSteppingFactorList[indx_microSteppingFactorList]);

  Serial.print("Current micro-steps per rotation: ");
  Serial.println(microstepsPerRevolution);

  Serial.println("---------------------------------");

  Serial.print("Current max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second
  Serial.print("Setting max speed to ");
  Serial.println(50000 * microSteppingFactorList[indx_microSteppingFactorList]); // Returns the most recent speed in steps per second
  stepper.setMaxSpeed(50000 * microSteppingFactorList[indx_microSteppingFactorList]);
  Serial.print("New max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second

  Serial.print("Setting acceleration to ");
  Serial.println(10 * microSteppingFactorList[indx_microSteppingFactorList]);
  stepper.setAcceleration(10 * microSteppingFactorList[indx_microSteppingFactorList]);
  // No function to read back set acceleration
  // NOTICE: Calling setAcceleration() is expensive, since it requires a square root to be calculated.

  Serial.print("Setting position to reach in [usteps] to ");
  Serial.println(nbr_desiredRevolutions * microstepsPerRevolution);
  stepper.moveTo(nbr_desiredRevolutions * microstepsPerRevolution);
  Serial.print("New target position in [usteps] is ");
  Serial.println(stepper.targetPosition ());// Returns (long) the most recent target position in steps. Positive is clockwise from the 0 position.

  Serial.print("Current stepper position [usteps]: ");
  Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.

  Serial.println("---------------------------------");

}// END OF SETUP



// -------------------------- Loop --------------------------
void loop() 
{

// runSpeed() is capable of running faster than  run();

  Serial.print("Current stepper position [usteps]: ");
  Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.
  stepper.moveTo(nbr_desiredRevolutions *  microstepsPerRevolution);
  stepper.setSpeed(0.1 * microstepsPerRevolution);
  enableStepper(true);
  

while (stepper.distanceToGo() != 0)
{
  stepper.runSpeed();
//  Serial.print("Current stepper position [usteps]: ");
//  Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.
}

 Serial.println("We arrived at the end of the movement");

 Serial.print("Current stepper position [usteps]: ");
 Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.
 enableStepper(false);


  stepper.moveTo(-nbr_desiredRevolutions *  microstepsPerRevolution);
  stepper.setSpeed(0.1 * microstepsPerRevolution);
enableStepper(true);
  

while (stepper.distanceToGo() != 0) // Returns the distance from the current position to the target position in steps. Positive is clockwise from the current position.
{
  stepper.runSpeed();
  //Serial.print("Current stepper position [usteps]: ");
  //Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.
}

  Serial.println("We arrived at the end of the movement");

 Serial.print("Current stepper position [usteps]: ");
 Serial.println(stepper.currentPosition ()); // Returns (long) the current motor position in steps. Positive is clockwise from the 0 position.
 enableStepper(false);
 

  

}// END OF LOOP



/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/
//******************************************************************************************
void enableStepper(bool enableOrder)
{
  if (enableOrder)
  {
    Serial.println("User asked to ENABLE the stepper, be careful");
    Serial.print("Changing...");
    //digitalWrite(PIN_MOTOR_ENA,LOW); // Inverse logic (active low)
    digitalWrite(PIN_MOTOR_ENA,HIGH); // Normal logic (active high)
    
  }
  else
  {
    Serial.println("User asked to DISABLE the stepper");
    Serial.print("Changing...");
    //digitalWrite(PIN_MOTOR_ENA,HIGH); // Inverse logic (active low)
    digitalWrite(PIN_MOTOR_ENA,LOW); // Normal logic (active high)
  }
  Serial.println("Done");
  
}


// END OF FILE
