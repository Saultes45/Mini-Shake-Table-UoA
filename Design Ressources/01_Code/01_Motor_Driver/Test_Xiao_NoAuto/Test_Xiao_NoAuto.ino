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

// Limit switch + mode switch + Sequencial Analog read trimpots
// These define's must be placed before #include "SAMDTimerInterrupt.h"
//#define TIMER_INTERRUPT_DEBUG         0
//#define _TIMERINTERRUPT_LOGLEVEL_     0
//#include "SAMDTimerInterrupt.h"
//#include "SAMD_ISR_Timer.h"

// Stepper motor driver
#include <AccelStepper.h>


// -------------------------- Defines --------------------------

#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

// Modes
//#define MODE_AUTO   1u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
//#define MODE_MANUAL 0u  // Use the 2 trimpots to describe the motion

// Stepper motor driver pins (logic low)
const uint8_t PIN_MOTOR_STEP   = 5; // Digital output 3.3V
const uint8_t PIN_MOTOR_DIR    = 4; // Digital output 3.3V
const uint8_t PIN_MOTOR_ENA    = 6; // Digital output 3.3V, manual logic

const uint16_t microSteppingFactorList[]    = {1, 4, 8, 16, 32, 64, 128, 256, 5, 10, 20, 25, 40, 50, 100, 200}; // Handled exclusively by the motor driver - I know, it is not sorted ascendedly but what can I do?
const uint8_t indx_microSteppingFactorList = 0; // Says to this program which physical microSteppingFactor is currently on the motor driver
const uint8_t nativePulsesPerRevolution = 200; // This parameter is from the motor and CANNOT be changed
const uint8_t microstepsPerRevolution = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;

// Since it can be changed during a "limit switch" or "end travel" distance
//  calibration, this parameter cannot be const and #define is a bad idea
// BOTH paramters have to be updated during a calibration
// TODO: only choose 1
//double angleToDistance = 2.0*3.14159265359*5.0/360.0; // [mm/°] // <------ TODO: This is a random value
//double distanceToAngle = 1/angleToDistance; // [°/mm]


uint16_t nbr_desiredRevolutions = 1;


// Frequency scale factor
// Asmplitude scale factor

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
//uint8_t crnt_Mode = MODE_MANUAL;
//uint8_t prev_Mode = MODE_MANUAL;

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

	//  // Limit switches pins
	//  pinMode(PIN_LIMIT_RIGHT, INPUT);
	//  pinMode(PIN_LIMIT_LEFT, INPUT);
	//
	//
	//  // Trimpot pins
	//  pinMode(PIN_TRMPT_FREQ, INPUT);
	//  pinMode(PIN_TRMPT_AMPL, INPUT);


	// initialize digital pin PIN_MOTOR_ENA as an output.
	pinMode(PIN_MOTOR_ENA, OUTPUT);

	// Disable the motor (logic inverse)
	digitalWrite(PIN_MOTOR_ENA,HIGH);


	Serial.println("---------------------------------");
	//Serial.print(F("\nStarting stepper motor test on ")); Serial.println(BOARD_NAME);
	//Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
	//Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));


	//  Serial.println("Reading frequency trimpot");
	//  // read the value from the sensor:
	//  int sensor1Value = analogRead(PIN_TRMPT_FREQ);
	//  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
	//  float voltage1 = sensor1Value * (3.3 / 1023.0);
	//  // print out the value you read:
	//  //Serial.println(voltage1);
	//  Serial.println(sensor1Value);
	//
	//    Serial.println("Reading amplitude trimpot");
	//  // read the value from the sensor:
	//  int sensor2Value = analogRead(PIN_TRMPT_AMPL);
	//  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
	//  float voltage2 = sensor1Value * (3.3 / 1023.0);
	//  // print out the value you read:
	//  //Serial.println(voltage1);
	//  Serial.println(sensor2Value);

	Serial.print("Current micro-steps settings: ");
	Serial.println(microSteppingFactorList[indx_microSteppingFactorList]);

	Serial.print("Current micro-steps per rotation: ");
	Serial.println(microstepsPerRevolution);


	Serial.print("Setting max speed to ");
	Serial.println();
	stepper.setMaxSpeed(50000 * microSteppingFactorList[indx_microSteppingFactorList]);
	Serial.print("New max speed is ");
	Serial.println();

	Serial.print("Setting max speed to ");
	Serial.println();
	stepper.setAcceleration(10000 * microSteppingFactorList[indx_microSteppingFactorList]);
	Serial.print("New max speed is ");
	Serial.println();

	Serial.print("Setting position to reach ");
	Serial.println();
	stepper.moveTo(nbr_desiredRevolutions *  microSteppingFactorList[indx_microSteppingFactorList]);


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
