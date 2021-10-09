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

void IR::targetSearch(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser) {
    // Reset stepper positions for logic
    lstepper.setCurrentPosition(0);
    rstepper.setCurrentPosition(0);
    turret.setCurrentPosition(0);

    // Reset stepper movement to constant velocity
    // TODO:: Check if this is constant acceleration or not
    lstepper.setAcceleration(MAX_SPEED_FULLSTEP);
    rstepper.setAcceleration(MAX_SPEED_FULLSTEP);

    float currAngle = 0;
    uint8_t BASE_STEP_INTERVAL = 12; // provides samples at 2deg
    float SCALING = (BASE_STEP_INTERVAL * 360 / STEPS_PER_REV_FULLSTEP);

    float maxlongReading = 0;
    float currReading = 0;
    long lstepperPos = 0;
    long rstepperPos = 0;
    while (1) {
        currReading = IR::totalSensorAvg(this->samples);
        if (currReading > maxlongReading) {
            maxlongReading = currReading;
            lstepperPos = lstepper.currentPosition();
            rstepperPos = rstepper.currentPosition();
        }
        stepCW(lstepper, rstepper, BASE_STEP_INTERVAL);

        if (currAngle >= 100) { // Tune
            // First check if the maxValue is a valid max value
            if (maxlongReading >= 20) { // TODO: tune this condition
                lstepper.moveTo(lstepperPos);
                rstepper.moveTo(rstepperPos);
                lstepper.enableOutputs();
                rstepper.enableOutputs();
                while (1) {
                    lstepper.run();
                    rstepper.run();
                    if (!lstepper.run() && !rstepper.run()) {
                        break;
                    }
                }
                lstepper.disableOutputs();
                rstepper.disableOutputs();

                // Do lateral search
                IR::lateralSearch(turret, laser);
            }
            lstepper.setCurrentPosition(0);
            rstepper.setCurrentPosition(0);
            currAngle = 0;
            continue;
        }
        currAngle += SCALING;
    }
}

void IR::lateralSearch2(AccelStepper& turret, Laser laser) {
    uint8_t TURRET_STEP_INTERVAL = 5; // Provides samples at 0.8deg
    float TURRET_SCALING = ((float)TURRET_STEP_INTERVAL * 360 / STEPS_PER_REV_HALFSTEP);
    float currTurretAngle = 0;
    long maxReadingPos = 0;
    float maxReading = 0;
    float currReading = 0;
    uint8_t latReadingCount = 0;

    //laser.constantOn();
    while (1) {
        currReading = IR::totalSensorAvg(this->samples);
        if (currReading > maxReading) {
            maxReading = currReading;
            maxReadingPos = turret.currentPosition();
        }
        moveTurret(turret, TURRET_STEP_INTERVAL);
        if (currTurretAngle >= 20) { // TODO: Tune this condition
            // now we move to the max value angle and fire laser
            turret.enableOutputs(); 
            turret.moveTo(maxReadingPos - 90);
            turret.runToPosition();
            turret.disableOutputs();
            laser.shootLaser();
            break;
        }
        currTurretAngle += TURRET_SCALING;
        latReadingCount++;
    }
    // Move turret back to home position
    turret.enableOutputs();
    turret.moveTo(0);
    turret.runToPosition();
    turret.disableOutputs();
}

void IR::lateralSearch(AccelStepper& turret, Laser laser) {
    float currTurretAngle = 0;
    long maxReadingPos = 0;
    float maxReading = 0;
    float currReading = 0;

    // First want to move to home configuration and then sweep 50 degrees
    uint8_t sweepAngle = 50;
    uint8_t lowThreshold = 10;

    homeTurret(turret);
    turret.enableOutputs();
    turret.move((long) sweepAngle * STEPS_PER_REV_HALFSTEP / 360);
    while(turret.run()) {
        /* Could even make a threshold here where it does not sweep after a certain decrease */
        currReading = IR::totalSensorAvg(this->samples);
        if (currReading > maxReading) {
            maxReading = currReading;
            maxReadingPos = turret.currentPosition();
        }
        if (maxReading - lowThreshold > currReading) {
            break;
        }
    }
    turret.moveTo(maxReadingPos);
    turret.runSpeedToPosition();
    turret.disableOutputs();
    moveTurretSensePose(turret);
}

void IR::targetSearchv2(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser) {
    // Reset stepper positions for logic
    lstepper.setCurrentPosition(0);
    rstepper.setCurrentPosition(0);
    turret.setCurrentPosition(0);
    moveTurretSensePose(turret);

    // Reset stepper movement to constant velocity
    lstepper.setAcceleration(MAX_SPEED_FULLSTEP);
    rstepper.setAcceleration(MAX_SPEED_FULLSTEP);

    uint8_t BASE_STEP_INTERVAL = 57; // provides samples at 10deg

    float currAngle = 0;
    float stepAngle = IR::calculateStepAngle(BASE_STEP_INTERVAL);

    float maxlongReading = 0;
    float currReading = 0;
    float threshold = 1.5; // TODO: tune this condition
    long lstepperPos = 0;
    long rstepperPos = 0;
    uint8_t fineStepActive = 0;
    while (1) {
        currReading = IR::totalSensorAvg(this->samples);
        // TODO: Make the robot move faster when small increases are seen
        if (currReading > maxlongReading) {
            /*
            if (abs(currReading - maxlongReading) < threshold && !fineStepActive) {
                BASE_STEP_INTERVAL = 100;
                stepAngle = IR::calculateStepAngle(BASE_STEP_INTERVAL);
            } else {
                BASE_STEP_INTERVAL = 57;
                stepAngle = IR::calculateStepAngle(BASE_STEP_INTERVAL);
                fineStepActive = 1;
            }
            */
            maxlongReading = currReading;
            lstepperPos = lstepper.currentPosition();
            rstepperPos = rstepper.currentPosition();
        }
        stepCW(lstepper, rstepper, BASE_STEP_INTERVAL);

        if (currAngle >= 90) {
            // First check if the maxValue is a valid max value
            if (maxlongReading >= 20) { // TODO: tune this condition
                // Move to first max pos
                multiMoveTo(lstepper, rstepper, lstepperPos, rstepperPos);
                // Re-sweep area to find accurate pos of maximum 
                slowSweep(lstepper, rstepper, 10); // TODO: tune this condition
                // Do lateral search
                IR::lateralSearch(turret, laser);
                rotateCW(lstepper, rstepper, 45);
                lstepperPos = 0;
                rstepperPos = 0;
                maxlongReading = 0;
                // Reset rotation steps to max after shooting
                //BASE_STEP_INTERVAL = 100; 
                //stepAngle = IR::calculateStepAngle(BASE_STEP_INTERVAL);
                //fineStepActive = 0;
            }
            lstepper.setCurrentPosition(0);
            rstepper.setCurrentPosition(0);
            currAngle = 0;
            continue;
        }
        currAngle += stepAngle;
    }
}

void IR::slowSweep(AccelStepper& lstepper, AccelStepper& rstepper, float sweep) {
    long middlePosL = lstepper.currentPosition();
    long middlePosR = rstepper.currentPosition();

    uint8_t BASE_STEP_INTERVAL = 6; // provides samples at 1deg

    float currAngle = 0;
    float stepAngle = (BASE_STEP_INTERVAL * 360 / STEPS_PER_REV_FULLSTEP);

    float maxlongReading = 0;
    float currReading = 0;
    long lstepperPos = 0;
    long rstepperPos = 0;

    // we rotate CCW sweep deg
    while (1) {
        currReading = IR::totalSensorAvg(this->samples);
        if (currReading > maxlongReading) {
            maxlongReading = currReading;
            lstepperPos = lstepper.currentPosition();
            rstepperPos = rstepper.currentPosition();
        }
        stepCCW(lstepper, rstepper, BASE_STEP_INTERVAL);

        if (currAngle >= sweep) {
            // Move to the middle position again
            currAngle = 0;
            multiMoveTo(lstepper, rstepper, middlePosL, middlePosR);
            break;
        }
        currAngle += stepAngle;
    }

    // we rotate CW sweep deg
    while (1) {
        currReading = IR::totalSensorAvg(this->samples);
        if (currReading > maxlongReading) {
            maxlongReading = currReading;
            lstepperPos = lstepper.currentPosition();
            rstepperPos = rstepper.currentPosition();
        }
        stepCW(lstepper, rstepper, BASE_STEP_INTERVAL);

        if (currAngle >= sweep) {
            // Move to maximum irridance position 
            currAngle = 0;
            multiMoveTo(lstepper, rstepper, lstepperPos, rstepperPos);
            // Step a couple degress CW due to slight error
            break;
        }
        currAngle += stepAngle;
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
    turret.move((long) _angle * STEPS_PER_REV_HALFSTEP / 360);
    while(turret.run());
    turret.disableOutputs();
}

void homeTurret(AccelStepper& turret) {
    turret.enableOutputs();
    turret.moveTo(0);
    turret.runToPosition();
    turret.disableOutputs();
}