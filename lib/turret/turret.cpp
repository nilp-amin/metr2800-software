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

    float currAngle = 0;
    uint8_t BASE_STEP_INTERVAL = 12; // provides samples at 2deg
    float SCALING = (BASE_STEP_INTERVAL * 360 / STEPS_PER_REV_FULLSTEP);

    float maxlongReading = 0;
    float currReading = 0;
    float lstepperPos = 0;
    float rstepperPos = 0;
    while (1) {
        currReading = IR::totalSensorAvg();
        if (currReading > maxlongReading) {
            maxlongReading = currReading;
            lstepperPos = lstepper.currentPosition();
            rstepperPos = rstepper.currentPosition();
        }
        stepCW(lstepper, rstepper, BASE_STEP_INTERVAL);

        if (currAngle >= 100) { // Tune
            // First check if the maxValue is a valid max value
            if (maxlongReading >= 10) { // TODO: tune this condition
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

void IR::lateralSearch(AccelStepper& turret, Laser laser) {
    uint8_t TURRET_STEP_INTERVAL = 24; // Provides samples at 0.5deg 6
    float TURRET_SCALING = ((float)TURRET_STEP_INTERVAL * 360 / STEPS_PER_REV_HALFSTEP);
    float currTurretAngle = 0;
    long maxReadingPos = 0;
    float maxReading = 0;
    float currReading = 0;
    uint8_t latReadingCount = 0;

    while (1) {
        currReading = IR::totalSensorAvg();
        if (currReading > maxReading) {
            maxReading = currReading;
            maxReadingPos = turret.currentPosition();
        }
        moveTurret(turret, TURRET_STEP_INTERVAL);
        if (currTurretAngle >= 20) { // TODO: Tune this condition
            // now we move to the max value angle and fire laser
            turret.enableOutputs(); 
            turret.moveTo(maxReadingPos - 100);
            turret.runToPosition();
            turret.disableOutputs();
            laser.shootLaser();
            break;
        }
        currTurretAngle += TURRET_SCALING;
        latReadingCount++;
    }
    turret.enableOutputs();
    turret.moveTo(0);
    turret.runToPosition();
    turret.disableOutputs();
}

float IR::totalSensorAvg() {
    float sum = 0;
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        float val = IR::readIR(pins[i]);
        this->readings[i] = val;
        sum += val;
    }
    return sum / (sizeof(pins)/sizeof(pins[0]));
}

float IR::readIR(uint8_t pin) {
    int total = 0;
    for (uint16_t i = 0; i < this->samples; i++)  {
        total += analogRead(pin);
    }
    return (total / this->samples);
}

void moveTurret(AccelStepper& turret, long step) {
    turret.enableOutputs();
    turret.move(step);
    while(turret.run());
    turret.disableOutputs();
}