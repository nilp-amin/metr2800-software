#include <Arduino.h>
#include <Ultrasonic.h>
#include <AccelStepper.h>

#include "../lib/navigation/navigation.h" 
#include "../lib/motor/motor.h"
#include "../lib/turret/turret.h"

Laser laser(LASER_PIN, 2000L);
IR irSensors(IR1_PIN, IR2_PIN, IR3_PIN, IR4_PIN,
             IR5_PIN, IR6_PIN, IR7_PIN, IR8_PIN, 
             100, 100);

Ultrasonic frontUltrasonic(F_TRIGGER, F_ECHO);
Ultrasonic rearUltrasonic(R_TRIGGER, R_ECHO);

// Have to flip pin 2 and 3 in decleration due to library
// Have to enable this class bfr running and disable when not running
AccelStepper leftStepper(AccelStepper::FULL4WIRE, L_STEPPER_PIN_1, 
                         L_STEPPER_PIN_3, L_STEPPER_PIN_2, L_STEPPER_PIN_4, false);
AccelStepper rightStepper(AccelStepper::FULL4WIRE, R_STEPPER_PIN_1, 
                          R_STEPPER_PIN_3, R_STEPPER_PIN_2, R_STEPPER_PIN_4, false);
// Use half step for speed and low current usage
AccelStepper turretStepper(AccelStepper::HALF4WIRE, TURRET_PIN_1, 
                           TURRET_PIN_3, TURRET_PIN_2, TURRET_PIN_4, false);
// AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_PC7, PIN_PC5, PIN_PC6, PIN_PC4);

void setup() {  
  //Serial.begin(9600);
  leftStepper.setMaxSpeed(MAX_SPEED_FULLSTEP);
  leftStepper.setAcceleration(500);

  rightStepper.setMaxSpeed(MAX_SPEED_FULLSTEP);
  rightStepper.setAcceleration(500);

  turretStepper.setMaxSpeed(MAX_SPEED_HALFSTEP);
  turretStepper.setAcceleration(200);
  turretStepper.setMaxSpeed(1000);
  turretStepper.setCurrentPosition(0);
  turretStepper.move(1000);
  pinMode(PIN_PD5, OUTPUT);
}

void lateralSearch(IR& ir, AccelStepper& turret, Laser laser) {
    uint8_t TURRET_STEP_INTERVAL = 24; // Provides samples at 0.5deg 6
    float TURRET_SCALING = ((float)TURRET_STEP_INTERVAL * 360 / STEPS_PER_REV_HALFSTEP);
    float currTurretAngle = 0;
    long maxReadingPos = 0;
    float maxReading = 0;
    float currReading = 0;
    uint8_t latReadingCount = 0;
    while (1) {
        currReading = ir.totalSensorAvg();
        if (currReading > maxReading) {
            maxReading = currReading;
            maxReadingPos = turretStepper.currentPosition();
            //laser.shootLaser();
            //Serial.println("Max Pos");
            //Serial.println(maxReadingPos);
        }
        moveTurret(turret, TURRET_STEP_INTERVAL);
        if (currTurretAngle >= 20) {
            // now we move to the max value angle and fire laser
            //Serial.println(moveTurretSteps);
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
    turretStepper.enableOutputs();
    turretStepper.moveTo(0);
    turretStepper.runToPosition();
    turretStepper.disableOutputs();
}

void loop() {
    if (digitalRead(PIN_PD5)) {
        laser.shootLaser();
  //need to scan area and move to centre
  //moveForward(leftStepper, rightStepper, 60);
  //rotateCCW(leftStepper, rightStepper, 360);
        locate(frontUltrasonic, rearUltrasonic, leftStepper, rightStepper);
  //lateralSearch(irSensors, turretStepper, laser);
  //irSensors.targetSearch(leftStepper, rightStepper, turretStepper, laser);
  //Serial.println(irSensors.totalSensorAvg());
  //delay(100);
    }
}