#include "turret.h"

Laser::Laser(uint8_t sigPin, uint8_t onTime) {
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
       uint16_t sensativity, uint16_t samples) {
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

void IR::targetSearch(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret) {
    // Reset stepper positions for logic
    lstepper.setCurrentPosition(0);
    rstepper.setCurrentPosition(0);
    turret.setCurrentPosition(0);

    float currAngle = 0;
    uint8_t readingCount = 0;
    uint8_t BASE_STEP_INTERVAL = 3;  // Provides samples at 0.5deg
    uint8_t TURRET_STEP_INTERVAL = 3; // Provides sampels at 0.5deg
    float SCALING = (BASE_STEP_INTERVAL * 360 / STEPS_PER_REV_FULLSTEP);
    // Obtain average samples readings every 0.5deg
    // Every 90 degrees find maximum reading and check difference relative to noise
    //  --> if contrast between noise and max reading > this->sensativity
    //  --> go to that step position or angle.
    //  --> then do a longitduinal search for largest spike
    // Every 90 degree reset everything to 0 and restart from first step again
    while (1) {
        IR::getReadings(readingCount);
        stepCW(lstepper, rstepper, BASE_STEP_INTERVAL);
        
        if (currAngle >= 90) {
            uint8_t isValid = 0;
            float maxValAngle = IR::zscoreAlgo(SCALING, isValid);
            if (isValid) {
                int moveSteps = (BASE_STEP_INTERVAL / SCALING) * (currAngle - maxValAngle);
                stepCCW(lstepper, rstepper, moveSteps);
                // Now we do a lateral search

                // Move slightly away from target then reset parameters
                rotateCW(lstepper, rstepper, 10);
            }
            currAngle = 0;
            readingCount = 0;
            lstepper.setCurrentPosition(0);
            rstepper.setCurrentPosition(0);
            memset(this->history, 0, sizeof(float)*sizeof(this->history));
            continue;
        }

        currAngle += SCALING;
        readingCount++;
    }
}

// naive implementation
float IR::zscoreAlgo(float scaling, uint8_t& isValid) {
    float max = 0;
    float currVal = 0;
    uint8_t maxIndex = 0;
    float baseline = IR::historyAvg();
    // We first find max value and its corresponding index
    for (uint8_t i = 0; i < sizeof(this->history); i++) {
        currVal = this->history[i];
        if (currVal > max) {
            max = currVal;
            maxIndex = i;
        }
    }

    // Now we check if it is actually a valid spike
    float stddev = IR::stddevHistory(baseline);
    if ((max - baseline) > stddev*this->sensativity) {
        isValid = 1;
    }

    return maxIndex * scaling;
}

// TODO: Finish
uint8_t IR::noiseyPeakFindingAlgo(uint8_t scaling) {
    uint8_t maxIndex = 0;
    float baseline = IR::historyAvg();
    float smoothed[sizeof(this->history)];
    memset(smoothed, 0, sizeof(float)*sizeof(smoothed));
    // Smooth data

    // Then find peaks
    return maxIndex;
}

float IR::historyAvg() {
    int sum = 0;
    float smoothed[sizeof(this->history)];
    memset(smoothed, 0, sizeof(float)*sizeof(this->history));
    for (uint8_t i = 0; i < sizeof(this->history); i++) {
        sum += this->history[i];
    }
    return sum / sizeof(this->history);
}

float IR::stddevHistory(float mean) {
    float stddev = 0.0;
    for (uint8_t i = 0; i < sizeof(this->history); i++) {
        stddev += pow(this->history[i] - mean, 2);
    }
    return sqrt(stddev / sizeof(this->history));
}

float IR::totalSensorAvg() {
    float sum = 0;
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        float val = IR::readIR(pins[i]);
        this->readings[i] = val;
        sum += val;
    }
    return sum / sizeof(pins);
}

void IR::getReadings(uint8_t anglePos) {
    this->history[anglePos] = IR::totalSensorAvg();
}

float IR::readIR(uint8_t pin) {
    int total = 0;
    for (uint16_t i = 0; i < this->samples; i++)  {
        total += analogRead(pin);
    }
    return (total / this->samples);
}

float IR::tvalues(bool inner=false) {
    uint8_t count = 2;
    int sum = this->readings[0] + this->readings[1];
    if (inner) {
        sum += this->readings[4] + this->readings[5];
        count += 2;
    }
    return (sum / count); // change division number if more included
}

float IR::bvalues(bool inner=false) {
    uint8_t count = 2;
    int sum = this->readings[2] + this->readings[3];
    if (inner) {
        sum += this->readings[6] + this->readings[7];
        count += 2;
    }
    return (sum / count);
}

float IR::rvalues(bool inner=false) {
    uint8_t count = 2;
    int sum = this->readings[1] + this->readings[2];
    if (inner) {
        sum += this->readings[5] + this->readings[6];
        count += 2;
    }
    return (sum / count);
}

float IR::lvalues(bool inner=false) {
    uint8_t count = 2;
    int sum = this->readings[0] + this->readings[3];
    if (inner) {
        sum += this->readings[4] + this->readings[7];
        count += 2;
    }
    return (sum / count);
}