#include <Arduino.h>

#define A PIN_PC7
#define B PIN_PC6
#define C PIN_PC5
#define D PIN_PC4

#define NUMBER_OF_STEPS_PER_REV 512 // 1 step in this case is 2048/4 --> less precise
#define t_time 2 // most good speed to torque

void setup(){
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(C,OUTPUT);
  pinMode(D,OUTPUT);
}

void write(int a,int b,int c,int d){
  digitalWrite(A,a);
  digitalWrite(B,b);
  digitalWrite(C,c);
  digitalWrite(D,d);
}

void onestep(float t){
  // We are doing 4 full step mode steps here
  write(1,0,0,0);
  delay(t);
  write(1,1,0,0);
  delay(t);
  write(0,1,0,0);
  delay(t);
  write(0,1,1,0);
  delay(t);
  write(0,0,1,0);
  delay(t);
  write(0,0,1,1);
  delay(t);
  write(0,0,0,1);
  delay(t);
  write(1,0,0,1);
  delay(t);
}

void backstep(float t){
  // We are doing 4 full step mode steps here
  write(0,0,0,1);
  delay(t);
  write(0,0,1,1);
  delay(t);
  write(0,0,1,0);
  delay(t);
  write(0,1,1,0);
  delay(t);
  write(0,1,0,0);
  delay(t);
  write(1,1,0,0);
  delay(t);
  write(1,0,0,0);
  delay(t);
  write(1,0,0,1);
  delay(t);
}

void loop(){
  int i;
  i=0;
  int count = (NUMBER_OF_STEPS_PER_REV / 360) * 180;
  while(i<count){
    onestep((float) 5);
    i++;
  }
  delay(1000);
  
  i = 0;
  while(i<count){
    backstep((float) 3);
    i++;
  }
  digitalWrite(A,LOW);
  digitalWrite(B,LOW);
  digitalWrite(C,LOW);
  digitalWrite(D,LOW);
  delay(1000);
}