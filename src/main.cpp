#include <Arduino.h>
#include <Ultrasonic.h>
#include <AccelStepper.h>

#include "../lib/navigation/navigation.h" 
#include "../lib/motor/motor.h"


Ultrasonic frontUltrasonic(F_TRIGGER, F_ECHO);
Ultrasonic rearUltrasonic(R_TRIGGER, R_ECHO);

// Have to flip pin 2 and 3 in decleration due to library
AccelStepper leftStepper(AccelStepper::FULL4WIRE, L_STEPPER_PIN_1, L_STEPPER_PIN_3, L_STEPPER_PIN_2, L_STEPPER_PIN_4);
AccelStepper rightStepper(AccelStepper::FULL4WIRE, R_STEPPER_PIN_1, R_STEPPER_PIN_3, R_STEPPER_PIN_2, R_STEPPER_PIN_4);
// Use half step for speed and low current usage
AccelStepper turretStepper(AccelStepper::HALF4WIRE, TURRET_PIN_1, TURRET_PIN_3, TURRET_PIN_2, TURRET_PIN_4);
// AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_PC7, PIN_PC5, PIN_PC6, PIN_PC4);

void setup() {  
  Serial.begin(9600);
}

void loop() {
  ;
}