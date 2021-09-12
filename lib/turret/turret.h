#ifndef TURRET_H_
#define TURRET_H_

#include <Arduino.h>
#include <string.h>
#include <math.h>

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
#define LATIDUINAL              1

class Laser {
    public:
        Laser(uint8_t sigPin, uint8_t onTime);
        void shootLaser();
    
    private:
        uint8_t trig;
        uint8_t _delay;
};

class IR {
    public:
        IR(uint8_t ir1, uint8_t ir2, uint8_t ir3, uint8_t ir4,
           uint8_t ir5, uint8_t ir6, uint8_t ir7, uint8_t ir8, 
           float sensativity, uint16_t samples);
        float totalSensorAvg();
        void targetSearch(AccelStepper& lstepper, AccelStepper& rstepper, AccelStepper& turret, Laser laser);

        float readings[8];

    private:
        uint16_t minReading = 100;
        uint16_t samples = 100;
        uint8_t pins[8];
        uint16_t stepAngle = 0;
        float sensativity = 3.19F; // calculated using stddev table
        float noiseReading = 0.0F;
        float history[180]; // Reset this every 90deg [0, 90]
        float latHistory[120]; // Reset this every time target located [0, 60]

        float readIR(uint8_t pin);
        float tvalues(bool inner=false);
        float bvalues(bool inner=false);
        float rvalues(bool inner=false);
        float lvalues(bool inner=false);
        float historyAvg();
        float maxLatHistory(uint8_t scaling);
        float stddevHistory(float mean, uint8_t lateral);
        void getReadings(uint8_t anglePos, uint8_t lateral);
        float zscoreAlgo(float scaling, uint8_t& valid);
};

#endif /*TURRET_H_ */