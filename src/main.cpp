#include <Arduino.h>
#include  <Ultrasonic.h>
#include <AccelStepper.h>

#define STEPS_PER_REV_HALFSTEP        4096 
#define MAX_SPEED_HALFSTEP            1000 // Reduce for more torque --> less speed

#define STEPS_PER_REV_FULLSTEP        2048
#define MAX_SPEED_FULLSTEP            500 // Reduce for more torque  --> less speed

Ultrasonic ultrasonic(PIN_PD0, PIN_PD2);
AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_PC7, PIN_PC5, PIN_PC6, PIN_PC4);

void setup() {  
  Serial.begin(9600);
}
void loop() {
  ;
}