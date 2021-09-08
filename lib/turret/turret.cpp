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

// Might want to store history of sensor readings
void IR::getReadings() {
    for (uint8_t i = 0; i < sizeof(pins); i++) {
        readings[i] = IR::readIR(pins[i]);
    }
}

float IR::readIR(uint8_t pin) {
    int total = 0;
    for (uint16_t i = 0; i < this->samples; i++)  {
        total += analogRead(pin);
    }
    return (total / this->samples);
}