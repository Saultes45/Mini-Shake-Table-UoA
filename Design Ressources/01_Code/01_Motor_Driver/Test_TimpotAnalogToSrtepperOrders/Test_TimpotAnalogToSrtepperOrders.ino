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
const uint16_t NBR_SAMPLES_MEDIAN = 200;
const float XIAO_ADC_MAX_LSB   = 1023.0; // <<--- This MUST BE float!!!!!

// Frequency
uint16_t median1 = 0;
uint16_t sensorValue1 = 0;
MedianFilter2<int> medianFilter1(NBR_SAMPLES_MEDIAN);
float current_trimpotFrequency_filtered = 0.0;

// Amplitude
uint16_t median2 = 0;
uint16_t sensorValue2 = 0;
MedianFilter2<int> medianFilter2(NBR_SAMPLES_MEDIAN);
float current_trimpotAmplitude_filtered = 0.0;

// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V

// Trimpots deadbands ([LSB] or [V] or [frequency]) -> LSB
const uint16_t TRMPT_DEADBAND_FREQ    = 100;
const uint16_t TRMPT_DEADBAND_AMPL    = 500;


// precision  (trimpot_frequency_max - trimpot_frequency_min) / 1024
// 0.97Hz/LSB
float trimpot_frequency_range_max_lsb = XIAO_ADC_MAX_LSB;
float trimpot_frequency_range_min_lsb = 6.0;
float trimpot_frequency_max = 1000; // in [Hz]
float trimpot_frequency_min = 0.0; // in [Hz]
// to use with stepper.setAcceleration

// amplitude  (trimpot_amplitude_max - trimpot_amplitude_min) / 1024
// 9.765625E-4 stepper shaft rotations/LSB
float trimpot_frequency_range_max_lsb = XIAO_ADC_MAX_LSB;
float trimpot_frequency_range_min_lsb = 6.0;
float trimpot_amplitude_max = 0.5; // in [stepper shaft rotations]
float trimpot_amplitude_min = 0.0; // in [stepper shaft rotations]
// to use with : stepper.moveTo

void setup() 
{
  Serial.begin(CONSOLE_BAUD_RATE);
}


void loop() 
{
  // read the input on analog pin 0:
  sensorValue1 = analogRead(PIN_TRMPT_FREQ);
  // Apply deadband
  if (sensorValue1 <= TRMPT_DEADBAND_FREQ)
  {
    // Then we are in the deadband and the outpout value should be 0[LSB]
    sensorValue1 = 0;
    }
  median1 = medianFilter1.AddValue(sensorValue1);
  current_trimpotFrequency_filtered = (median1-trimpot_frequency_range_min_lsb) * (trimpot_frequency_max - trimpot_frequency_min)/(trimpot_frequency_range_max_lsb-trimpot_frequency_range_min_lsb) + trimpot_frequency_min;

  
  // read the input on analog pin 1:
  sensorValue2 = analogRead(PIN_TRMPT_AMPL);
    // Apply deadband
  if (sensorValue2 <= TRMPT_DEADBAND_AMPL)
  {
    // Then we are in the deadband and the outpout value should be 0[LSB]
    sensorValue2 = 0;
    }
  median2 = medianFilter2.AddValue(sensorValue2);
  current_trimpotAmplitude_filtered = (median2-trimpot_amplitude_range_min_lsb) * (trimpot_amplitude_max - trimpot_amplitude_min)/(trimpot_amplitude_range_max_lsb-trimpot_amplitude_range_min_lsb) + trimpot_amplitude_min;

// Raw values
  Serial.print(sensorValue1);
  Serial.print(" ");
  Serial.print(median1);
  Serial.print(" ");
  Serial.print(sensorValue2);
  Serial.print(" ");
  Serial.println(median2);

// Prints

//Serial.print(current_trimpotFrequency_filtered);
//Serial.print(" ");
//Serial.println(current_trimpotAmplitude_filtered);
  
  
  
  delay(1);        // delay in between reads for stability
}
