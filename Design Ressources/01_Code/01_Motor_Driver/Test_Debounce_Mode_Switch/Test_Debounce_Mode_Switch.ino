/*
button_int.ino

*/

#define BUTTON  8        // Can be any I/O pin from 0 to 10
#define DEBOUNCE_TIME 10 // millisecond button debouncing time

volatile bool flagMode = false;
volatile unsigned long last_interrupt_time = 0;

void buttonISR()
{
	noInterrupts();
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > 200)
	{
		flagMode = true;
		digitalWrite(LED_BUILTIN, digitalRead(BUTTON));
	}
	last_interrupt_time = interrupt_time;
	//}
	//delay(DEBOUNCE_TIME);
	interrupts();
}

void setup() {
	// initialize the LED pin as an output:
	pinMode(LED_BUILTIN, OUTPUT);
	// initialize the pushbutton pin as an input and enable the internal pull up resistor:
	pinMode(BUTTON, INPUT);
	attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, CHANGE);
}

void loop() {
	// empty right now, add code as needed.
	if (flagMode)
	{
		flagMode = false;
		Serial.print("Mode change with time: ");
		Serial.println(millis());
	}
}
