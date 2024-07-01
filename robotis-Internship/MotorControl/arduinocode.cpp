#include <Servo.h>

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

const int motor1 = 3;//thumb
const int motor2 = 5;
const int motor3 = 6;
const int motor4 = 9;
const int motor5 = 10;
int pos=0;
void setup() {
  Servo1.attach(motor1);
  Servo2.attach(motor2);
  Servo3.attach(motor3);
  Servo4.attach(motor4);
  Servo5.attach(motor5);
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
    for (int i=0; i<60; i++){
      Servo1.write(60-i);
      delay(20);
    }
    for (int i=0; i<60; i++){
      Servo1.write(i);
      delay(20);
    }
  }
}