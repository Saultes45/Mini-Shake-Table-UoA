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


//#define USE_TC3


#ifdef USE_TC3
#include <TimerTC3.h>
#else
#include <TimerTCC0.h>
#endif



#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console


// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V



bool isLEDOn = false;


void timerIsr()
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
	digitalWrite(13, isLEDOn);
	isLEDOn = !isLEDOn;

	// Trimpot read and process
	//-------------------------
	
	// read the input on analog pin 0:
	sensorValue1 = analogRead(PIN_TRMPT_FREQ);


	// read the input on analog pin 1:
	sensorValue2 = analogRead(PIN_TRMPT_AMPL);


	// Display (for  <DEBUG> only)
	//----------------------------

	Serial.print(sensorValue1);
	Serial.print(" ");
	Serial.println(sensorValue2);
	
}


void setup()
{

	Serial.begin(CONSOLE_BAUD_RATE);
	pinMode(13, OUTPUT);

#ifdef USE_TC3
	TimerTc3.initialize(10000);
	TimerTc3.attachInterrupt(timerIsr);
#else
	TimerTcc0.initialize(10000); // 1e6 = 1s
	TimerTcc0.attachInterrupt(timerIsr);
#endif


}

void loop() {


}
