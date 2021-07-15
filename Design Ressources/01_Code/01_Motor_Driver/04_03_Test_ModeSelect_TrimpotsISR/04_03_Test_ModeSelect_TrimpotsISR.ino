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


#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

#define PIN_TOGGLE_MODE  8        // Can be any I/O pin from 0 to 10
#define DEBOUNCE_TIME 50 // millisecond button debouncing time
// Modes
#define MODE_AUTO   0u  // Wait orders and scenarios from the Raspberry pi through USB (UART)
#define MODE_MANUAL 1u  // Use the 2 trimpots to describe the motion
//#define MODE_START  MODE_MANUAL // tells the program which mode should be enabled when starting
#define MODE_START  MODE_AUTO // tells the program which mode should be enabled when starting


// Trimpot pins
const uint8_t PIN_TRMPT_FREQ    = A0; // Analog input 3.3V
const uint8_t PIN_TRMPT_AMPL    = A1; // Analog input 3.3V




// -------------------------- Global variables ----------------
bool isLEDOn = false;

volatile bool           flagMode            = false; 
volatile unsigned long  last_interrupt_time = 0;

// Mode handling
//uint8_t crnt_Mode = MODE_MANUAL;
//uint8_t prev_Mode = MODE_MANUAL;


// -------------------------- ISR ----------------

//******************************************************************************************
void toggleSwitchModeISR() 
{
     noInterrupts();
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME) 
 {
    flagMode = true; 
    //digitalWrite(LED_BUILTIN, digitalRead(BUTTON)); 
 }
 last_interrupt_time = interrupt_time;
//}
     //delay(DEBOUNCE_TIME);
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

    Serial.print(sensorValue1);
    Serial.print(" ");
    Serial.println(sensorValue2);
     
}


// -------------------------- SetUp --------------------------
void setup() 
{

    Serial.begin(CONSOLE_BAUD_RATE);
    pinMode(LED_BUILTIN, OUTPUT);

  // initialize the LED pin as an output:
   
   // initialize the pushbutton pin as an input
   pinMode(PIN_TOGGLE_MODE, INPUT);
   attachInterrupt(digitalPinToInterrupt(PIN_TOGGLE_MODE), toggleSwitchModeISR, CHANGE); //toggleSwitchModeISR

     /* After this attachInterrupt, be careful not to toggle the 
      *  mode switch of the timers for the trimpots will never start
      */

//if (MODE_START == MODE_MANUAL)
//
//{
//  #ifdef USE_TC3
//      TimerTc3.initialize(10000);
//      TimerTc3.attachInterrupt(timerTrimpotISR);
//  #else
//      TimerTcc0.initialize(10000); // 1e6 = 1s
//      TimerTcc0.attachInterrupt(timerTrimpotISR);
//  #endif
//}

// Enable the flag virtually, just once at the start as to read the location of the toggle switch
flagMode = true;


}


// -------------------------- Loop --------------------------
void loop() 
{  
    if (flagMode)
    {
      flagMode = false; // Reset the flag immediatly

      // Since the toggle switch ISR is triggered on 
      //  change, read the final and debounced state
      if (digitalRead(PIN_TOGGLE_MODE) == MODE_MANUAL)
      {
        // If we are in Manual, then attach the timer interrupt to read the trimpots
        
        #ifdef USE_TC3
        TimerTc3.initialize(10000);
        TimerTc3.attachInterrupt(timerTrimpotISR);
        #else
        TimerTcc0.initialize(10000); // 1e6 = 1s
        TimerTcc0.attachInterrupt(timerTrimpotISR);
        #endif
      }
      else
      {
        // If we are in Auto or (Scenario), then detach the 
        //  timer interrupt to stop reading the trimpots

        #ifdef USE_TC3
        TimerTc3.initialize(10000);
        TimerTc3.detachInterrupt();
        #else
        TimerTcc0.initialize(10000); // 1e6 = 1s
        TimerTcc0.detachInterrupt();
        #endif
      
      }
      
      //Serial.print("Mode change with time: ");
      //Serial.println(millis());
    }
}
