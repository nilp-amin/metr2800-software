#include "motor.h"

// Private scope
int angleToSteps(uint8_t angle);

int angleToSteps(uint8_t angle) {
	return (angle * STEPS_PER_REV_FULLSTEP / 360);
}

int distanceToSteps(uint8_t distance) {
	// base off wheel diameter
	// radius wheel = 60mm
	return (int) (distance * STEPS_PER_REV_FULLSTEP)/(2 * M_PI * 3);
}

// Blocking code
void rotateCW(AccelStepper& left, AccelStepper& right, uint8_t angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(-steps);
	while (left.run() || right.run());
	left.disableOutputs();
	right.disableOutputs();
}

// Blocking code
void rotateCCW(AccelStepper& left, AccelStepper& right, uint8_t angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(-steps);
	right.move(steps);
	while (left.run() || right.run()); 
	left.disableOutputs();
	right.disableOutputs();
}

void moveForward(AccelStepper &left, AccelStepper &right, uint8_t distance) {
	int steps = distanceToSteps(distance);
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(steps);
	while (left.run() || right.run()); 
	left.disableOutputs();
	right.disableOutputs();
}

// Move and angle and distance
bool move(AccelStepper &left, AccelStepper &right, int angle, int distance) {
	// rotate angle, let + CW, - CCW
	if (angle > 0) {
		rotateCW(left, right, angle);
	} else if (angle < 0) {
		angle *= -1;
		rotateCCW(left, right, angle);
	}
	moveForward(left, right, distance);
	return true;
}

void stepCW(AccelStepper& left, AccelStepper& right, int steps) {
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(-steps);
	while (left.run() || right.run());
	left.disableOutputs();
	right.disableOutputs();
}

void stepCCW(AccelStepper& left, AccelStepper& right, int steps) {
	left.enableOutputs();
	right.enableOutputs();
	left.move(-steps);
	right.move(steps);
	while (left.run() || right.run());
	left.disableOutputs();
	right.disableOutputs();
}