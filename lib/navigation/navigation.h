#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <Arduino.h>
#include <Ultrasonic.h>

#include <stdio.h>
#include <math.h>


#define F_ECHO                  PIN_PD3
#define F_TRIGGER               PIN_PD4
#define R_ECHO                  PIN_PD2
#define R_TRIGGER               PIN_PD6 


float distanceCentre(int x, int y);
float angleCentre(int x, int y);
void locate();

#endif /*NAVIGATION_H_ */