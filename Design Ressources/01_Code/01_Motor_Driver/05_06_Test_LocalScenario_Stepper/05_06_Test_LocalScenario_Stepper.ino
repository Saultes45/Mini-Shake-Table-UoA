/* ========================================
*
* Copyright University of Auckland Ltd, 2021
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   :
* Creation date : 2021-07-17
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
* findout why the the first movement is always 1/2, I don't know why
* 
* 
* ========================================
*/

// -------------------------- Includes --------------------------
#include <AccelStepper.h>

// -------------------------- Defines --------------------------

#define SERIAL_VERBOSE
#define CONSOLE_BAUD_RATE             115200  // Baudrate in [bauds] for serial communication to the console

// Limit switches pins
//---------------------
const uint8_t PIN_LIMIT_RIGHT     = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT      = 10; // Digital input 3.3V
const uint16_t LS_DEBOUNCE_TIME   = 50;    //100 // millisecond button debouncing time


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
const long microSteppingFactorList[]                  = {1, 4, 8, 16, 32, 64, 128, 256, 5, 10, 20, 25, 40, 50, 100, 200}; 
/* Says to this program which physical microSteppingFactor 
 *  is currently on the motor driver 
 */
const uint8_t indx_microSteppingFactorList              = 0; 
const float   max_allowedMicroStepsPerSeconds           = 10000.0;//500.0 * 200.0; //10000.0; //10000000
const float   max_allowedMicroStepsPerSecondsPerSeconds = 14000.0;//200.0 * 200.0;//750.0; // //750
const long    nativePulsesPerRevolution                 = ((long)200); // This parameter is from the motor and CANNOT be changed
const long    microstepsPerRevolution                   = microSteppingFactorList[indx_microSteppingFactorList] * nativePulsesPerRevolution;

// Local Scenario
//----------------
const int   nbr_movementsScenario                 = 10; // This variable is const because it defuines the size of the memory we can allocate to scenario parameters
      long  scenarioSteps[nbr_movementsScenario]  = {0};
      float scenarioSpeed[nbr_movementsScenario]  = {0.0};
      bool  abortScenario                         = true; // a variable that store potential reason to stop a scenario


 
// -------------------------- Global variables ----------------

// Limit switches L+R
//--------------------
volatile bool flagLS_r   = false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;
volatile bool flagLS_l   = false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;


/* Define a stepper and the pins it will use, and precise that
 *  we use an external dedicated stepper driver, thus we have 2 pins: 
 *  STEP and DIR (ENA is handled manually) 
 */
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);

// -------------------------- Functions declaration --------------------------
void enableStepper(bool enableOrder);




// -------------------------- ISR [2] ----------------

//******************************************************************************************
void LW_r_ISR()
{
  noInterrupts();
  unsigned long interrupt_time = millis();
  // If interrupts come faster than LS_DEBOUNCE_TIME, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time_ls_r > LS_DEBOUNCE_TIME)
  {
    enableStepper(false); // disable the stepper ASAP, can be anbaled back in the loop
    abortScenario = true; // make sure no other stepper movement are executed
    flagLS_r = true;
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_RIGHT));
    if(digitalRead(PIN_LIMIT_RIGHT))
    {
      stateLS_r = true;
      //enableTrimpots(false);
    }
    else
    {
      stateLS_r = false;
      //enableTrimpots(true); // enable back only if in Manual
    }
   last_interrupt_time_ls_r = interrupt_time;
  }
  
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
    enableStepper(false); // disable the stepper ASAP, can be anbaled back in the loop
    abortScenario = true; // make sure no other stepper movement are executed
    flagLS_l = true;
    digitalWrite(LED_BUILTIN, digitalRead(PIN_LIMIT_LEFT));
    if(digitalRead(PIN_LIMIT_LEFT))
    {
      stateLS_l = true;
      //enableTrimpots(false);
    }
    else
    {
      stateLS_l = false;
      //enableTrimpots(true); // enable back only if in Manual
    }
   last_interrupt_time_ls_l = interrupt_time;
  }
  
  interrupts();
}



// -------------------------- SetUp --------------------------
void setup() 
{

// scenarioSteps[0] = (+1 * microstepsPerRevolution); // the first one is always 1/2, I don't know why
// scenarioSteps[1] = -1 * microstepsPerRevolution;
// scenarioSteps[2] = +1 * microstepsPerRevolution;
// scenarioSteps[3] = -1 * microstepsPerRevolution;
// scenarioSteps[4] = +1 * microstepsPerRevolution;

 scenarioSteps[0] = +200; // the first one is always 1/2, I don't know why
 scenarioSteps[1] = -200;
 scenarioSteps[2] = +200;
 scenarioSteps[3] = -100;
 scenarioSteps[4] = +100;
 scenarioSteps[5] = +50;
 scenarioSteps[6] = -25;
 scenarioSteps[7] = +25;
 scenarioSteps[8] = -200;
 scenarioSteps[9] = +200;


 scenarioSpeed[0] = +250.0;
 scenarioSpeed[1] = -10000.0;
 scenarioSpeed[2] = +10000.0;
 scenarioSpeed[3] = -10000.0;
 scenarioSpeed[4] = +10000.0;
 scenarioSpeed[5] = +10000.0;
 scenarioSpeed[6] = -10000.0;
 scenarioSpeed[7] = +1000.0;
 scenarioSpeed[8] = -500.0;
 scenarioSpeed[9] = +25.0;


delay(5000);

  Serial.begin(CONSOLE_BAUD_RATE); //Begin serial communication (USB)
  Serial.println("---------------------------------");
  
  // initialize digital pin PIN_MOTOR_ENA as an output.
  pinMode(PIN_MOTOR_ENA, OUTPUT);

  // Disable the motor immediatly (external toggle switch should also be in disable positon)
  enableStepper(false);

    // Limit switches
    //----------------
   /* Attach each a different ISR
  *  For each we want to know when it is
  *  triggered and released so we can enable the drive again if necesary
  */
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LW_r_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LW_l_ISR, CHANGE);

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
  Serial.println(max_allowedMicroStepsPerSeconds); // Returns the most recent speed in steps per second
  stepper.setMaxSpeed(max_allowedMicroStepsPerSeconds);
  Serial.print("New max speed is ");
  Serial.println(stepper.speed()); // Returns the most recent speed in steps per second

  Serial.print("Setting acceleration to ");
  Serial.println(max_allowedMicroStepsPerSecondsPerSeconds);
  stepper.setAcceleration(max_allowedMicroStepsPerSecondsPerSeconds);
  // No function to read back set acceleration
  // NOTICE: Calling setAcceleration() is expensive, since it requires a square root to be calculated.

  Serial.println("---------------------------------");
  abortScenario = false;


}// END OF SETUP



// -------------------------- Loop --------------------------
void loop() 
{
  if (abortScenario == false)
  {
    enableStepper(true);
    delay(1000); // wait here for the lock or snap in place of the motor
  
    for (int cnt_scenarioMovements = 0 ; cnt_scenarioMovements < nbr_movementsScenario; cnt_scenarioMovements++)
    {
      stepper.setSpeed(scenarioSpeed[cnt_scenarioMovements]); // Order matters!!!! 1->speed 2->steps
      stepper.moveTo(scenarioSteps[cnt_scenarioMovements]);
      while ((stepper.distanceToGo() != 0) && (abortScenario == false))
      {
        stepper.run();
      }
    }
  
    enableStepper(false);
  }
  delay(1000);
  

}// END OF LOOP



/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/
//******************************************************************************************
void enableStepper(bool enableOrder)
{
  if (enableOrder)
  {
    if (digitalRead(PIN_LIMIT_RIGHT) == false & digitalRead(PIN_LIMIT_LEFT) == false && abortScenario == false)
    {
      Serial.print("ENABLING the stepper, be careful...");
      digitalWrite(PIN_MOTOR_ENA,LOW); // Inverse logic (active low)
//    digitalWrite(PIN_MOTOR_ENA,HIGH); // Normal logic (active high)
    }
  }
  else
  {
    Serial.print("DISABLING the stepper...");
    digitalWrite(PIN_MOTOR_ENA,HIGH); // Inverse logic (active low)
//    digitalWrite(PIN_MOTOR_ENA,LOW); // Normal logic (active high)
  }
  Serial.println("done");
  
}


// END OF FILE
