// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Stepper motor driver pins
const uint8_t PIN_MOTOR_STEP   = 2; // Digital output 3.3V
const uint8_t PIN_MOTOR_DIR    = 3; // Digital output 3.3V
const uint8_t PIN_MOTOR_ENA    = 4; // Digital output 3.3V, manual logic

uint16_t nbr_revolutions = 1;

//uint32_t scenarioPos

const uint8_t stepFactor = 200;
const uint8_t pulsesPerRevolution = 200;
//const uint8_t NBR_SCENARIOS

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, PIN_MOTOR_STEP, PIN_MOTOR_DIR);

void setup()
{  

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize digital pin PIN_MOTOR_ENA as an output.
  pinMode(PIN_MOTOR_ENA, OUTPUT);

// Disable the motor (logic inverse)
  digitalWrite(PIN_MOTOR_ENA,HIGH);

  Serial.begin(115200);
  Serial.println("---------------------------------");
  Serial.println("Welcome to the stepper motor test");

  
  Serial.println("Setting max speed to ");
  stepper.setMaxSpeed(50000 * stepFactor);
  stepper.setAcceleration(10000 * stepFactor);
  stepper.moveTo(nbr_revolutions * pulsesPerRevolution * stepFactor);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
    {
      delay(3000);
      stepper.moveTo(-stepper.currentPosition());
    }
      

    stepper.run();
}
