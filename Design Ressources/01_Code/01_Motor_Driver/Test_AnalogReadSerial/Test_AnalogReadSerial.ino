/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/


#include "MedianFilterLib2.h"

uint16_t median1 = 0;
uint16_t sensorValue1 = 0;
MedianFilter2<int> medianFilter1(200);

uint16_t median2 = 0;
uint16_t sensorValue2 = 0;
MedianFilter2<int> medianFilter2(200);


void setup() 
{
  Serial.begin(115200);
}


void loop() 
{
  // read the input on analog pin 0:
  sensorValue1 = analogRead(A0);
  median1 = medianFilter1.AddValue(sensorValue1);


  
  // read the input on analog pin 0:
  sensorValue2 = analogRead(A1);
  median2 = medianFilter2.AddValue(sensorValue2);

  Serial.print(sensorValue1);
  Serial.print(" ");
  Serial.print(median1);
  Serial.print(" ");
  Serial.print(sensorValue2);
  Serial.print(" ");
  Serial.println(median2);
  
  
  
  delay(1);        // delay in between reads for stability
}
