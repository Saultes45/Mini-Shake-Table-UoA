/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-15
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
#define USE_TC3


#ifdef USE_TC3
#include <TimerTC3.h>
#else
#include <TimerTCC0.h>
#endif

// -------------------------- Defines --------------------------

// General
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console


// Limit switches pins
const uint8_t PIN_LIMIT_RIGHT   = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT    = 10; // Digital input 3.3V
#define LS_DEBOUNCE_TIME 100 // millisecond button debouncing time

// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V
const uint32_t FREQUENCY_READ_TRIMPOTS = 10000; // in [Hz]
const uint16_t NBR_SAMPLES_MEDIAN = 200;
const float XIAO_ADC_MAX_LSB   = 1023.0; // <<--- This MUST BE float!!!!!
// Trimpots deadbands ([LSB] or [V] or [frequency]) -> LSB
const uint16_t TRMPT_DEADBAND_FREQ    = 25;
const uint16_t TRMPT_DEADBAND_AMPL    = 25;

// Mode selection
#define PIN_TOGGLE_MODE  	8    // Can be any I/O pin from 0 to 10
#define DEBOUNCE_TIME 		50      // millisecond button debouncing time

// Modes
#define MODE_AUTO   0u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
#define MODE_MANUAL 1u  // Use the 2 trimpots to execute the motion



// -------------------------- Global variables ----------------


// Limit switches
volatile bool flagLS_r 	= false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;

volatile bool flagLS_l 	= false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;

// Mode toggle switch
bool 					isLEDOn 			= false;
volatile bool           flagMode            = false;
volatile unsigned long  last_interrupt_time = 0;

// Trimpot: Frequency
uint16_t median1 		= 0;
uint16_t sensorValue1 	= 0;
MedianFilter2<int> medianFilter1(NBR_SAMPLES_MEDIAN);
float current_trimpotFrequency_filtered = 0.0;

// Trimpot: Amplitude
uint16_t median2 		= 0;
uint16_t sensorValue2 	= 0;
MedianFilter2<int> medianFilter2(NBR_SAMPLES_MEDIAN);
float current_trimpotAmplitude_filtered = 0.0;

// precision  (trimpot_frequency_max - trimpot_frequency_min) / 1024
// 0.97Hz/LSB
float trimpot_frequency_range_max_lsb 	= XIAO_ADC_MAX_LSB; // in [LSB]
float trimpot_frequency_range_min_lsb 	= 6.0;  			// in [LSB]
float trimpot_frequency_max 			= 1000; 			// in [Hz]
float trimpot_frequency_min 			= 0.0; 				// in [Hz]
// to use with stepper.setAcceleration

// amplitude  (trimpot_amplitude_max - trimpot_amplitude_min) / 1024
// 9.765625E-4 stepper shaft rotations/LSB
float trimpot_amplitude_range_max_lsb 	= XIAO_ADC_MAX_LSB;  // in [LSB]
float trimpot_amplitude_range_min_lsb 	= 6.0;   			 // in [LSB]
float trimpot_amplitude_max 			= 0.5; 				 // in [stepper shaft rotations]
float trimpot_amplitude_min 			= 0.0; 				 // in [stepper shaft rotations]
// to use with : stepper.moveTo

// -------------------------- ISR ----------------

//******************************************************************************************
void LW_r_ISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time_ls_r > LS_DEBOUNCE_TIME)
	{
		flagLS_r = true;
		digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_RIGHT));
		if(digitalRead(PIN_LIMIT_RIGHT))
		{
			stateLS_r = true;
		}
		else
		{
			stateLS_r = false;
		}
	}
	last_interrupt_time_ls_r = interrupt_time;
	interrupts();
}

//******************************************************************************************
void LW_l_ISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time_ls_l > LS_DEBOUNCE_TIME)
	{
		flagLS_l = true;
		digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_RIGHT));
		if(digitalRead(PIN_LIMIT_LEFT))
		{
			stateLS_l = true;
		}
		else
		{
			stateLS_l = false;
		}
	}
	last_interrupt_time_ls_l = interrupt_time;
	interrupts();
}

//******************************************************************************************
void toggleSwitchModeISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME)
	{
		flagMode = true;
	}
	last_interrupt_time = interrupt_time;
	interrupts();
}

//******************************************************************************************
void timerTrimpotISR()
{

	/* NOTICE:
*  Do NOT put noInterupts(); here because this is a low priority and long execution time "task"
*  Limit switches IRS should be able to trigger in there
*/

	// Local variable declaration
	//----------------------------
	uint16_t sensorValue1 = 0;
	uint16_t sensorValue2 = 0;


	// Visual indication
	//-------------------
	digitalWrite(LED_BUILTIN, isLEDOn);
	isLEDOn = !isLEDOn;

	// Trimpot read and process
	//-------------------------

	// read the input on analog pin 0:
	sensorValue1 = analogRead(PIN_TRMPT_FREQ);


	// read the input on analog pin 1:
	sensorValue2 = analogRead(PIN_TRMPT_AMPL);


	// Display (for  <DEBUG> only)
	//----------------------------

	//    Serial.print(sensorValue1);
	//    Serial.print(" ");
	//    Serial.println(sensorValue2);

}



// -------------------------- SetUp --------------------------
void setup()
{

	Serial.begin(CONSOLE_BAUD_RATE);


	// initialize the LED pin as an output:
	pinMode(LED_BUILTIN, OUTPUT);

	// initialize the limit switches pins as an input
	pinMode(PIN_LIMIT_RIGHT, INPUT);
	pinMode(PIN_LIMIT_LEFT,  INPUT);

	/* Attach each a different ISR
	*  For each we want to know when it is
	*  triggered and released so we can enable the drive again if necesary
	*/
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LW_r_ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LW_l_ISR, CHANGE);


	// initialize the pushbutton pin as an input
	pinMode(PIN_TOGGLE_MODE, INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_TOGGLE_MODE), toggleSwitchModeISR, CHANGE); //toggleSwitchModeISR

	/* After this attachInterrupt, be careful not to toggle the
	*  mode switch of the timers for the trimpots will never start
	*/

	// Enable the flag virtually, just once at the start as to read the location of the toggle switch
	flagMode = true;

}







// -------------------------- Loop --------------------------
void loop()
{


	// Checking the flag of all the ISRs [3]
	//--------------------------------------

	if (flagLS_r)
	{
		flagLS_r = false;
		Serial.printf("RIGHT Limit Switch ISR triggered %d, current status: %d\r\n", millis(), stateLS_r);
	}
	if (flagLS_l)
	{
		flagLS_l = false;
		Serial.printf("LEFT Limit Switch ISR triggered %d, current status: %d\r\n", millis(), stateLS_l);
	}

	if (flagMode)
	{
		flagMode = false; // Reset the flag immediatly

		// Since the toggle switch ISR is triggered on
		//  change, read the final and debounced state
		if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
		{
			// If we are in Manual, then attach the timer interrupt to read the trimpots

			#ifdef USE_TC3
			TimerTc3.initialize(FREQUENCY_READ_TRIMPOTS);
			TimerTc3.attachInterrupt(timerTrimpotISR);
			#else
			TimerTcc0.initialize(FREQUENCY_READ_TRIMPOTS); // 1e6 = 1s
			TimerTcc0.attachInterrupt(timerTrimpotISR);
			#endif
		}
		else
		{
			// If we are in Auto or (Scenario), then detach the
			//  timer interrupt to stop reading the trimpots

			#ifdef USE_TC3
			TimerTc3.initialize(FREQUENCY_READ_TRIMPOTS);
			TimerTc3.detachInterrupt();
			#else
			TimerTcc0.initialize(FREQUENCY_READ_TRIMPOTS); // 1e6 = 1s
			TimerTcc0.detachInterrupt();
			#endif
		}
	}
}


// END OF THE FILE
