#include "turret.h"

Laser::Laser(uint8_t sigPin, long onTime) {
     trig = sigPin;
     _delay = onTime;
     pinMode(trig, OUTPUT);
}

void Laser::shootLaser() {
    digitalWrite(trig, HIGH);
    delay(_delay);
    digitalWrite(trig, LOW);
}

void Laser::constantOn() {
    digitalWrite(trig, HIGH);
}

IR::IR(uint8_t ir1, uint8_t ir2, uint8_t ir3, uint8_t ir4,
       uint8_t ir5, uint8_t ir6, uint8_t ir7, uint8_t ir8, 
       float sensativity, uint16_t samples) {
    this->minReading = sensativity;
    this->samples = samples;

    this->pins[0]= ir1;
    this->pins[1] = ir2;
    this->pins[2] = ir3;
    this->pins[3] = ir4;
    this->pins[4] = ir5;
    this->pins[5] = ir6;
    this->pins[6] = ir7;
    this->pins[7] = ir8;
}

void IR::lateralSearch(AccelStepper& turret, Laser laser) {
    long maxReadingPos = 0;
    float maxReading = 0;
    float currReading = 0;

    // First want to move to home configuration and then sweep 50 degrees
    uint8_t sweepAngle = 40;
    uint8_t lowThreshold = 30;

    homeTurret(turret);
    laser.constantOn();
    turret.enableOutputs();
    turret.move((long) sweepAngle * STEPS_PER_REV_HALFSTEP / 360);
    while(turret.run()) {
        currReading = IR::totalSensorAvg(this->samples);
        //currReading = IR::innerSensorAvg(this->samples);
        if (currReading > maxReading) {
            maxReading = currReading;
            maxReadingPos = turret.currentPosition();
        }
        if (maxReading - lowThreshold > currReading) { // TODO: Fix this threshold before testing with it again
            //break;
        }
    }
    turret.moveTo(maxReadingPos - 100);
    turret.runToPosition();
    turret.disableOutputs();
    laser.shootLaser();
    moveTurretSensePose(turret);
}

void IR::targetSearch(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser) {
    lstepper.setCurrentPosition(0);
    rstepper.setCurrentPosition(0);
    lstepper.setSpeed(MAX_SPEED_FULLSTEP);
    rstepper.setSpeed(-MAX_SPEED_FULLSTEP);
    turret.setCurrentPosition(0);
    moveTurretSensePose(turret);

    //const unsigned long angle = 30 * 360;
    //const long steps = (long)angle * STEPS_PER_REV_FULLSTEP / 360;
    const long checkBackSteps = (long)90 * STEPS_PER_REV_FULLSTEP / 360;
    //lstepper.move(steps);
    //rstepper.move(-steps);

    float maxReading = 0;
    float currReading = 0;
    long lstepperPos = 0;
    long rstepperPos = 0;
    long checkBackStepCount = 0;
    uint8_t stepCount = 0;
    while(1) {
        if (lstepper.runSpeed()) {
            checkBackStepCount++;
        }
        rstepper.runSpeed();
        if (stepCount == 10) {
            currReading = IR::totalSensorAvg(this->samples);
            if (currReading > maxReading) {
                maxReading = currReading;
                lstepperPos = lstepper.currentPosition(); 
                rstepperPos = rstepper.currentPosition();
            }
            if (checkBackStepCount >= checkBackSteps && maxReading > 20) {
                multiMoveTo(lstepper, rstepper, lstepperPos - 10, rstepperPos + 10);
                IR::lateralSearch(turret, laser);
                rotateCW(lstepper, rstepper, 45);
                lstepperPos = 0;
                rstepperPos = 0;
                maxReading = 0;
                checkBackStepCount = 0;
                lstepper.setSpeed(MAX_SPEED_FULLSTEP);
                rstepper.setSpeed(-MAX_SPEED_FULLSTEP);
                //lstepper.move(steps);
                //rstepper.move(-steps);
            } else if (checkBackStepCount >= checkBackSteps) {
                checkBackStepCount = 0;
            }
            stepCount = 0;
            continue;
        }
        stepCount++;
    }
}

float IR::calculateStepAngle(uint8_t interval) {
    return (interval * 360 / STEPS_PER_REV_FULLSTEP);
}

float IR::totalSensorAvg(uint16_t _samples) {
    float sum = 0;
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        float val = IR::readIR(pins[i], _samples);
        this->readings[i] = val;
        sum += val;
    }
    return sum / (sizeof(pins)/sizeof(pins[0]));
}

float IR::innerSensorAvg(uint16_t _samples) {
    float sum = IR::readIR(pins[4], _samples) +
                IR::readIR(pins[5], _samples) +
                IR::readIR(pins[6], _samples) + 
                IR::readIR(pins[7], _samples);
    return sum / 4;
}

float IR::readIR(uint8_t pin, uint16_t _samples) {
    int total = 0;
    for (uint16_t i = 0; i < _samples; i++)  {
        total += analogRead(pin);
    }
    return (total / _samples);
}

/* We could read IR array as we step*/
void moveTurret(AccelStepper& turret, long step) {
    turret.enableOutputs();
    turret.move(step);
    while(turret.run());
    turret.disableOutputs();
}

/* Sense position is at 30 degrees */
void moveTurretSensePose(AccelStepper& turret) {
    uint8_t _angle = 30;
    turret.enableOutputs();
    turret.moveTo((long) _angle * STEPS_PER_REV_HALFSTEP / 360);
    turret.runToPosition();
    turret.disableOutputs();
}

void homeTurret(AccelStepper& turret) {
    turret.enableOutputs();
    turret.moveTo(0);
    turret.runToPosition();
    turret.disableOutputs();
}