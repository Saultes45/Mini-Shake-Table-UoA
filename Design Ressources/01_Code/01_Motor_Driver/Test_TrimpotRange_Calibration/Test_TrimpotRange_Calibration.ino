/*
AnalogReadSerial

Reads an analog input on pin 0, prints the result to the Serial Monitor.
Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

This example code is in the public domain.

https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/


#include "MedianFilterLib2.h"


// -------------------------- Defines --------------------------

#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console


// Analog trimpots

// Frequency
uint16_t sensorValue1 = 10; // Do not start at a min value, that might skew the results
uint16_t sensorValue1_max = 0;
uint16_t sensorValue1_min = 1023;

// Amplitude
uint16_t sensorValue2 = 10; // Do not start at a min value, that might skew the results
uint16_t sensorValue2_max = 0;
uint16_t sensorValue2_min = 1023;

// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V


void setup()
{
	Serial.begin(CONSOLE_BAUD_RATE);
}


void loop()
{
	// read the input on analog pin 0:
	sensorValue1 = analogRead(PIN_TRMPT_FREQ);
	if(sensorValue1 < sensorValue1_min)
	{
		sensorValue1_min = sensorValue1;
	}
	if(sensorValue1 > sensorValue1_max)
	{
		sensorValue1_max = sensorValue1;
	}
	// read the input on analog pin 1:
	sensorValue2 = analogRead(PIN_TRMPT_AMPL);
	if(sensorValue2 < sensorValue2_min)
	{
		sensorValue2_min = sensorValue2;
	}
	if(sensorValue2 > sensorValue2_max)
	{
		sensorValue2_max = sensorValue2;
	}


	// Raw values
	Serial.print(sensorValue1);
	Serial.print(" ");
	Serial.print(sensorValue1_min);
	Serial.print(" ");
	Serial.print(sensorValue1_max);
	Serial.print(" ");
	Serial.print(sensorValue2);
	Serial.print(" ");
	Serial.print(sensorValue2_min);
	Serial.print(" ");
	Serial.println(sensorValue2_max);



	delay(10);        // delay in between reads for stability
}
