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

    uint8_t angle = 360;
    uint8_t angleStep = 5;
    while (1) {
        getReadings();
        // Check spikes in 20 degree sectors of 5deg steps 
        // If spike seen in 20 degree sectors investigate
        rotateCW(lstepper, rstepper, angleStep);
    }
    
}

// Might want to store history of sensor readings
void IR::getReadings() {
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        this->readings[i] = IR::readIR(pins[i]);
    }
    history[this->sectorCount % 4] = irrdance();
    this->sectorCount++;
}

float IR::irrdance() {
    int sum = 0;
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        sum += this->readings[i];
    }
    return (sum/ sizeof(pins));
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