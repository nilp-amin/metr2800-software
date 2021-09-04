#include <Arduino.h>
#include  <Ultrasonic.h>

Ultrasonic ultrasonic(PIN_PD0, PIN_PD2);

void setup() {
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor HC-SR04 Test");
}

void loop() {
    Serial.println(ultrasonic.read());
    delay(10);
}