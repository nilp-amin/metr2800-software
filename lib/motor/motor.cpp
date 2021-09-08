#include "motor.h"

// Private scope
int angleToSteps(uint8_t angle);

int angleToSteps(uint8_t angle) {
	return (angle * STEPS_PER_REV_FULLSTEP / 360);
}

// Blocking code
void rotateCW(AccelStepper &left, AccelStepper &right, uint8_t angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(-steps);
	while (!left.run() || !right.run());
	left.disableOutputs();
	right.disableOutputs();
}

// Blocking code
void rotateCCW(AccelStepper &left, AccelStepper &right, uint8_t angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(-steps);
	right.move(steps);
	while (!left.run() || !right.run()); 
	left.disableOutputs();
	right.disableOutputs();
}