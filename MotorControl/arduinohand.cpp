#include <Servo.h>

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

const int motor1 = 3;
const int motor2 = 5;
const int motor3 = 6;
const int motor4 = 9;
const int motor5 = 10;

void setup() {
  Servo1.attach(motor1);
  Servo2.attach(motor2);
  Servo3.attach(motor3);
  Servo4.attach(motor4);
  Servo5.attach(motor5);
}

void loop() {
  //Servo4.write(360);
  //Servo4.write(180);
  Servo5.write(360);
  Servo5.write(0);
}