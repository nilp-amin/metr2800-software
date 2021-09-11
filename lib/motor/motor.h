#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>
#include <AccelStepper.h>

#define STEPS_PER_REV_HALFSTEP              4096 
#define MAX_SPEED_HALFSTEP                  1000 // Reduce for more torque --> less speed

#define STEPS_PER_REV_FULLSTEP              2048
#define MAX_SPEED_FULLSTEP                  500 // Reduce for more torque  --> less speed

#define WHEEL_RADIUS			            30 // In mm

// Left stepper motor
#define L_STEPPER_PIN_1			            PIN_PC7
#define	L_STEPPER_PIN_2			            PIN_PC6
#define L_STEPPER_PIN_3			            PIN_PC5
#define L_STEPPER_PIN_4			            PIN_PC4

// Right stepper motor
#define R_STEPPER_PIN_1			            PIN_PC0
#define R_STEPPER_PIN_2			            PIN_PC1
#define R_STEPPER_PIN_3			            PIN_PC2
#define R_STEPPER_PIN_4			            PIN_PC3

// Turret stepper motor
#define TURRET_PIN_1			            PIN_PB3
#define TURRET_PIN_2			            PIN_PB2
#define TURRET_PIN_3			            PIN_PB1
#define TURRET_PIN_4			            PIN_PB0


void rotateCW(AccelStepper& left, AccelStepper& right, uint8_t angle);
void rotateCCW(AccelStepper& left, AccelStepper& right, uint8_t angle);
void moveForward(AccelStepper& left, AccelStepper& right, uint8_t distance);
bool move(AccelStepper& left, AccelStepper& right, int angle, int distance);
void stepCW(AccelStepper& left, AccelStepper& right, int steps);
void stepCCW(AccelStepper& left, AccelStepper& right, int steps);

#endif /* MOTOR_H_ */