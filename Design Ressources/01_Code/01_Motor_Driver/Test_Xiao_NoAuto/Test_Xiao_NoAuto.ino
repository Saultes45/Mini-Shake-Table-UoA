/* ========================================
*
* Copyright University of Auckland, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault, Naser
* Verified by   : Nathanaël Esnault
* Creation date : 2021-05-19
* Version       : 1.0 (finished on 2021-??-??)
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
* Xiao pinout
* Xiao microcontroller:     https://wiki.seeedstudio.com/Seeeduino-XIAO/
* AccelStepper:             http://www.airspayce.com/mikem/arduino/AccelStepper/Blocking_8pde-example.html
*                           https://github.com/waspinator/AccelStepper
*                         
* Github project (public):  https://github.com/Saultes45/Mini-Shake-Table-UoA
* PCB EsayEDA (private):    https://easyeda.com/account/project/setting/basic?project=f569670f797046d5bf32fa1eb878fbe5               
* Motor driver:             https://www.jss-motor.com/product_category/Stepper-motor-drivers.html
*                           https://www.jss-motor.com/product/DM860A-2-phase-stepper-motor-driver.html
*
* TODO
* Use FreeRTOS and tasks instead of linear logic
* Find a way to transfer data from/to raspberry pi
* Limit Switch calibration 
*
* ========================================
*/

// -------------------------- Includes --------------------------
// Stepper motor driver
#include <AccelStepper.h>


// -------------------------- Defines --------------------------

#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

// Modes
#define MODE_AUTO   1u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
#define MODE_MANUAL 0u  // Use the 2 trimpots to describe the motion

// Stepper motor driver pins (logic low)
const uint8_t PIN_MOTOR_STEP   = 5; // Digital output 3.3V
const uint8_t PIN_MOTOR_DIR    = 4; // Digital output 3.3V
const uint8_t PIN_MOTOR_ENA    = 6; // Digital output 3.3V, manual logic

const uint16_t microSteppingFactorList[]    = {1, 200}; // Handled exclusively by the motor driver
const uint8_t indx_microSteppingFactorList = 1; // Says to this program which physical microSteppingFactor is currently on the motor driver
const uint8_t microSteppingFactor = 200;
const uint8_t pulsesPerRevolution = 200 * microSteppingFactor;


uint16_t nbr_revolutions = 1;

// Scenarios handling
//const uint8_t NBR_SCENARIOS

// Limit switches pins
const uint8_t PIN_LIMIT_RIGHT   = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT    = 10; // Digital input 3.3V

// Mode selection
const uint8_t PIN_MODE          = 8; // Digital input 3.3V

// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V



// -------------------------- Global variables ----------------

// Scenarios handling
//uint32_t scenarioPos

// Mode handling
uint8_t crnt_Mode = MODE_MANUAL;
uint8_t prev_Mode = MODE_MANUAL;

// Define a stepper and the pins it will use
// Precise that we use a dedicated stepper driver, thus we have 2 pins: STEP and DIR (ENA is handled manually)
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);

// -------------------------- ISR ----------------

// -------------------------- Functions declaration --------------------------


// -------------------------- Set up --------------------------
void setup()
{  

	Serial.begin(CONSOLE_BAUD_RATE); //Begin serial communication (USB)
	pinMode(LED_BUILTIN, OUTPUT);   //Set up the on-board LED (RED, close to the uUSB port)

	// Indicate the start of the setup with the red on-board LED
	//------------------------------------------------------------
	digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED ON
	delay(10);                         // Wait a bit


  // Limit switches pins
  pinMode(PIN_LIMIT_RIGHT, INPUT);
  pinMode(PIN_LIMIT_LEFT, INPUT);


  // Trimpot pins
  pinMode(PIN_TRMPT_FREQ, INPUT);
  pinMode(PIN_TRMPT_AMPL, INPUT);


  // initialize digital pin PIN_MOTOR_ENA as an output.
  pinMode(PIN_MOTOR_ENA, OUTPUT);

  // Disable the motor (logic inverse)
  digitalWrite(PIN_MOTOR_ENA,HIGH);

  Serial.begin(115200);
  Serial.println("---------------------------------");
  Serial.println("Welcome to the stepper motor test");

  
  Serial.print("Setting max speed to ");
  Serial.println();
  stepper.setMaxSpeed(50000 * microSteppingFactor);
  Serial.print("New max speed is ");
  Serial.println();

  Serial.print("Setting max speed to ");
  Serial.println();
  stepper.setAcceleration(10000 * microSteppingFactor);
  Serial.print("New max speed is ");
  Serial.println();

  Serial.print("Setting position to reach ");
  Serial.println();
  stepper.moveTo(nbr_revolutions * pulsesPerRevolution * microSteppingFactor);


} // END OF SETUP

// -------------------------- Loop --------------------------
void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
    {
      // Pause for 3s
      delay(3000);
      // Prepare the stepper to go the other way
      stepper.moveTo(-stepper.currentPosition());
    }
      

    stepper.run();
}



/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/
//******************************************************************************************


// END OF FILE
