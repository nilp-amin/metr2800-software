#ifndef TURRET_H_
#define TURRET_H_

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <AccelStepper.h>

#include "../motor/motor.h"

#define LASER_PIN               PIN_PB4     


/*
    (1)*          *(2)

        (5)*  *(6)
        (8)*  *(7)

    (4)*          *(3)
*/
#define IR1_PIN                 PIN_PA0
#define IR2_PIN                 PIN_PA4
#define IR3_PIN                 PIN_PA7
#define IR4_PIN                 PIN_PA3
#define IR5_PIN                 PIN_PA1
#define IR6_PIN                 PIN_PA5
#define IR7_PIN                 PIN_PA6
#define IR8_PIN                 PIN_PA2

#define LONGITDUINAL            0
#define LATITUDINAL             1

class Laser {
    public:
        Laser(uint8_t sigPin, long onTime);
        void shootLaser();
    
    private:
        uint8_t trig;
        long _delay;
};

class IR {
    public:
        IR(uint8_t ir1, uint8_t ir2, uint8_t ir3, uint8_t ir4,
           uint8_t ir5, uint8_t ir6, uint8_t ir7, uint8_t ir8, 
           float sensativity, uint16_t samples);
        float totalSensorAvg(uint16_t _samples);
        void targetSearch(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser);
        void targetSearchv2(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser);
        void lateralSearch(AccelStepper& turret, Laser laser);

        float readings[8];

    private:
        uint16_t minReading = 100;
        uint16_t samples = 100;
        uint8_t pins[8];

        float readIR(uint8_t pin, uint16_t _samples);
        void slowSweep(AccelStepper& lstepper, AccelStepper& rstepper, float sweep);
};

void moveTurret(AccelStepper& turret, long step);

#endif /*TURRET_H_ */