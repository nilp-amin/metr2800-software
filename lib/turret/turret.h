#ifndef TURRET_H_
#define TURRET_H_

#include <Arduino.h>

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
           uint16_t sensativity, uint16_t samples);
        void getReadings();
        float readings[8];

    private:
        uint16_t minReading;
        uint16_t samples;
        uint8_t pins[8];
        float readIR(uint8_t pin);
};

#endif /*TURRET_H_ */