#include <Servo.h>

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

const int motor1 = 3; //thumb
const int motor2 = 5; //index
const int motor3 = 6; //middle
const int motor4 = 9; //ring finger
const int motor5 = 10; //pinky
int motorposition[5] = {60,60,60,60,60}; // M1, M2, M3, M4, M5

void setup() {
  Servo1.attach(motor1);
  Servo2.attach(motor2);
  Servo3.attach(motor3);
  Servo4.attach(motor4);
  Servo5.attach(motor5);
  Serial.begin(9600);
}

void closehand(){ //fistbump, yes
    for (int i=1; i<=60; i++){
      if (motorposition[0] > 0) {
        motorposition[0]--;
        Servo1.write(motorposition[0]);
      }
      if (motorposition[1] > 0) {
      motorposition[1]--;
      Servo2.write(motorposition[1]);
      }
      if (motorposition[2] > 0) {
      motorposition[2]--;
      Servo3.write(motorposition[2]);
      }
      if (motorposition[3] > 0) {
      motorposition[3]--;
      Servo4.write(motorposition[3]);
      }
      if (motorposition[4] > 0) {
      motorposition[4]--;
      Servo5.write(motorposition[4]);
      }
      if (motorposition[0] == 0 && motorposition[1] == 0 && motorposition[2] == 0 && 
      motorposition[3] == 0 && motorposition[4] == 0) {
        break;
        }
      delay(20);
    }
  }

void openhand() { //high five, thank you
    for (int i=1; i<=60; i++){
      if (motorposition[0] < 60) {
        motorposition[0]++;
        Servo1.write(motorposition[0]);
      }
      if (motorposition[1] < 60) {
        motorposition[1]++;
        Servo2.write(motorposition[1]);
      }
      if (motorposition[2] < 60) {
        motorposition[2]++;
        Servo3.write(motorposition[2]);
      }
      if (motorposition[3] < 60) {
        motorposition[3]++;
        Servo4.write(motorposition[3]);
      }
      if (motorposition[4] < 60) {
        motorposition[4]++;
        Servo5.write(motorposition[4]);
      }
      if (motorposition[0] == 60 && motorposition[1] == 60 && motorposition[2] == 60 && 
      motorposition[3] == 60 && motorposition[4] == 60) {
        break;
        }
      delay(20);
    }
  }

void hello() { //starting open hand
  openhand();
  for (int i=1; i<=60; i++){ // thumb moving 60
      motorposition[0]--;
      Servo1.write(motorposition[0]);
      delay(20);
    }
  }

void handshake() { //starting open hand
  openhand();
  for (int i=1; i<=60; i++){
      if (i % 3 == 0) { //thumb moving 20
        motorposition[0]--;
        Servo1.write(motorposition[0]);
      }
      if (i % 6 == 0) { //index moving 10
        motorposition[1]--;
        Servo2.write(motorposition[1]);
      }
      if (i % 6 == 0) { //middle moving 10
        motorposition[2]--;
        Servo3.write(motorposition[2]);
      }
      if (i % 4 == 0) { //ring finger moving 15
        motorposition[3]--;
        Servo4.write(motorposition[3]);
      }
      if (i % 3 == 0) { // pinky moving 20
        motorposition[4]--;
        Servo5.write(motorposition[4]);
      }
      delay(7);
    }
  }

void count_to_5 () { // starting closehand
  closehand();
    for (int i=1; i<=60; i++){
      motorposition[1]++;
      Servo2.write(motorposition[1]);
      delay(20);
    }
    delay(500);
    for (int i=1; i<=60; i++){
      motorposition[2]++;
      Servo3.write(motorposition[2]);
      delay(20);
    }
    delay(500);
    for (int i=1; i<=60; i++){
      motorposition[3]++;
      Servo4.write(motorposition[3]);
      delay(20);
    }
    delay(500);
    for (int i=1; i<=60; i++){
      motorposition[4]++;
      Servo5.write(motorposition[4]);
      delay(20);
    }
    delay(500);
    for (int i=1; i<=60; i++){
      motorposition[0]++;
      Servo1.write(motorposition[0]);
      delay(20);
    }
  }

//void no () {
//    
//  }

void goodbye () { // starting openhand
    for (int i=1; i<=3; i++) {
      openhand();
      for (int i=1; i<=60; i++) {
        motorposition[1]--;
        Servo2.write(motorposition[1]);
        motorposition[2]--;
        Servo3.write(motorposition[2]);
        motorposition[3]--;
        Servo4.write(motorposition[3]);
        motorposition[4]--;
        Servo5.write(motorposition[4]);
        delay(20);
      }
    }
    openhand();
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