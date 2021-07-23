/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : Nathanaël Esnault
* Creation date : 2021-07-22
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

// Trimpots sequencing
//---------------------
#define USE_TC3

// Trimpots sequencing
#ifdef USE_TC3
#include <TimerTC3.h>
#else
#include <TimerTCC0.h>
#endif


// Trimpots filtering
//---------------------
#include "MedianFilterLib2.h"


// -------------------------- Defines --------------------------

// Trimpot pins
//--------------
const uint8_t PIN_TRMPT_FREQ            = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL            = A1; // Analog input 3.3V
const uint32_t FREQUENCY_READ_TRIMPOTS  = 10000; // in [Hz] 1000
const uint16_t NBR_SAMPLES_MEDIAN       = 50;
const float XIAO_ADC_MAX_LSB            = 1023.0; // <<--- This MUST be a float!!!!!

// Trimpots deadbands ([LSB] or [V] or [frequency]) -> LSB
//--------------------------------------------------------
const uint16_t TRMPT_DEADBAND_FREQ      = 10;
const uint16_t TRMPT_DEADBAND_AMPL      = 10;




// -------------------------- Global variables ----------------

volatile bool trimpotFlag = false; // Flag for the HW timer ISR to tell the main loop it is time to read the trimpots 

// Trimpot: Frequency
uint16_t median1          = 0;
uint16_t sensorValue1     = 0;
MedianFilter2<int> medianFilter1(NBR_SAMPLES_MEDIAN);
volatile float current_trimpotFrequency_filtered = 0.0;

// Trimpot: Amplitude
uint16_t median2        = 0;
uint16_t sensorValue2   = 0;
MedianFilter2<int> medianFilter2(NBR_SAMPLES_MEDIAN);
volatile float current_trimpotAmplitude_filtered = 0.0;

// precision  (trimpot_frequency_max - trimpot_frequency_min) / 1024
// 0.97Hz/LSB
float trimpot_frequency_range_max_lsb   = XIAO_ADC_MAX_LSB; // in [LSB]
float trimpot_frequency_range_min_lsb   = 6.0;        // in [LSB]
float trimpot_frequency_max             = 0.25;       // in [Hz]
float trimpot_frequency_min             = 0.0;        // in [Hz]
// to use with stepper.setAcceleration

// amplitude  (trimpot_amplitude_max - trimpot_amplitude_min) / 1024
// 9.765625E-4 stepper shaft rotations/LSB
float trimpot_amplitude_range_max_lsb   = XIAO_ADC_MAX_LSB;  // in [LSB]
float trimpot_amplitude_range_min_lsb   = 6.0;         // in [LSB]
float trimpot_amplitude_max             = 800.0;         // in [mm]
float trimpot_amplitude_min             = 0.0;         // in [mm]
// to use with : stepper.moveTo



// END OF THE FILE
