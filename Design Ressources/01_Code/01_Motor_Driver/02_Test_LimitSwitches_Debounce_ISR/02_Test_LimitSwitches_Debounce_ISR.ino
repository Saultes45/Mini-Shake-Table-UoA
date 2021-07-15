/*
button_int.ino

*/
// Limit switches pins
const uint8_t PIN_LIMIT_RIGHT   = 9;  // Digital input 3.3V
const uint8_t PIN_LIMIT_LEFT    = 10; // Digital input 3.3V
#define LS_DEBOUNCE_TIME 100 // millisecond button debouncing time



// -------------------------- Global variables ----------------

volatile bool flagLS_r 	= false; // this flag indicates a change to the main loop
volatile bool stateLS_r = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_r = 0;

volatile bool flagLS_l 	= false; // this flag indicates a change to the main loop
volatile bool stateLS_l = false; // this flag indicates if the LS is triggered to the main loop
volatile unsigned long last_interrupt_time_ls_l = 0;


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



// -------------------------- SetUp --------------------------
void setup() {
	// initialize the LED pin as an output:
	pinMode(LED_BUILTIN, OUTPUT);
	
	// initialize the limit switches pins as an input
	
	// Limit switches pins
	pinMode(PIN_LIMIT_RIGHT, INPUT);
	pinMode(PIN_LIMIT_LEFT, INPUT);

	/* Attach each a different ISR
	*  For each we want to know when it is
	*  triggered and released so we can enable the drive again if necesary
	*/
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_RIGHT), LW_r_ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_LIMIT_LEFT), LW_l_ISR, CHANGE);
}


// -------------------------- Loop --------------------------
void loop() {

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
}
