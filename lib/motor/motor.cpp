#include "motor.h"

// Private scope
long angleToSteps(float angle);

long angleToSteps(float angle) {
	return ((long)angle * STEPS_PER_REV_FULLSTEP / 360);
}

long distanceToSteps(int distance) {
	// base off wheel diameter
	// Radius of wheel = 30mm
	return  ((long)distance * STEPS_PER_REV_FULLSTEP)/(2 * M_PI * 3);
}

// Blocking code
void rotateCW(AccelStepper& left, AccelStepper& right, float angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(-steps);
    while (1) {
        left.run();
        right.run();
        if (!left.run() && !right.run()) {
            break;
        }
    }
	left.disableOutputs();
	right.disableOutputs();
}

// Blocking code
void rotateCCW(AccelStepper& left, AccelStepper& right, float angle) {
	int steps = angleToSteps(angle);
	left.enableOutputs();
	right.enableOutputs();
	left.move(-steps);
	right.move(steps);
    while (1) {
        left.run();
        right.run();
        if (!left.run() && !right.run()) {
            break;
        }
    }
	left.disableOutputs();
	right.disableOutputs();
}

void moveForward(AccelStepper &left, AccelStepper &right, int distance) {
	long steps = distanceToSteps(distance);
	left.enableOutputs();
	right.enableOutputs();
	left.move(steps);
	right.move(steps);
    while (1) {
        left.run();
        right.run();
        if (!left.run() && !right.run()) {
            break;
        }
    }
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
	while (1) {
        left.run();
        right.run();
        if (!left.run() && !right.run()) {
            break;
        }
    }
	left.disableOutputs();
	right.disableOutputs();
}

void stepCCW(AccelStepper& left, AccelStepper& right, int steps) {
	left.enableOutputs();
	right.enableOutputs();
	left.move(-steps);
	right.move(steps);
	while (1) {
        left.run();
        right.run();
        if (!left.run() && !right.run()) {
            break;
        }
	}  
	left.disableOutputs();
	right.disableOutputs();
}