#include <Servo.h>

// Servo objects
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;

// Motor pin assignments
const int motor1 = 3; // thumb
const int motor2 = 5; // index
const int motor3 = 6; // middle
const int motor4 = 9; // ring finger
const int motor5 = 10; // pinky
int startAngle = 45;
int angleChange = 90;
int maxAngle = startAngle + angleChange;
int offset[5] = {20, -12, 7, 2, -6};
int motorStartAngle[5] = {startAngle + offset[0], startAngle + offset[1], startAngle + offset[2],
 startAngle + offset[3], startAngle + offset[4]};
int motorMaxAngle[5] = {maxAngle + offset[0], maxAngle + offset[1], maxAngle + offset[2],
                        maxAngle + offset[3], maxAngle + offset[4]};
int motorposition[5] = {motorStartAngle[0], motorStartAngle[1], motorStartAngle[2],
                        motorStartAngle[3], motorStartAngle[4]}; // M1, M2, M3, M4, M5

void setup() {
  Servo1.attach(motor1);
  Servo2.attach(motor2);
  Servo3.attach(motor3);
  Servo4.attach(motor4);
  Servo5.attach(motor5);
  Serial.begin(9600);
} 

void closehand() { // fistbump, yes
  for (int i = 1; i <= angleChange; i++) {
    if (motorposition[0] < motorMaxAngle[0]) {
      motorposition[0]++;
      Servo1.write(motorposition[0]);
    }
    if (motorposition[1] < motorMaxAngle[1]) {
      motorposition[1]++;
      Servo2.write(motorposition[1]);
    }
    if (motorposition[2] < motorMaxAngle[2]){
      motorposition[2]++;
      Servo3.write(motorposition[2]);
    }
    if (motorposition[3] < motorMaxAngle[3]) {
      motorposition[3]++;
      Servo4.write(motorposition[3]);
    }
    if (motorposition[4] < motorMaxAngle[4]) {
      motorposition[4]++;
      Servo5.write(motorposition[4]);
    }
    if (motorposition[0] == motorMaxAngle[0] && motorposition[1] == motorMaxAngle[1]
     && motorposition[2]  == motorMaxAngle[2] && motorposition[3] == motorMaxAngle[3]
     && motorposition[4] == motorMaxAngle[4]) {
      break;
    }
    delay(10);
  }
}

void openhand() { // high five, thank you
  for (int i = 1; i <= angleChange; i++) {
    if (motorposition[0] > motorStartAngle[0]) {
      motorposition[0]--;
      Servo1.write(motorposition[0]);
    }
    if (motorposition[1] > motorStartAngle[1]) {
      motorposition[1]--;
      Servo2.write(motorposition[1]);
    }
    if (motorposition[2] > motorStartAngle[2]) {
      motorposition[2]--;
      Servo3.write(motorposition[2]);
    }
    if (motorposition[3] > motorStartAngle[3]) {
      motorposition[3]--;
      Servo4.write(motorposition[3]);
    }
    if (motorposition[4] > motorStartAngle[4]) {
      motorposition[4]--;
      Servo5.write(motorposition[4]);
    }
    if (motorposition[0] == motorStartAngle[0] && motorposition[1] == motorStartAngle[1] && motorposition[2] == motorStartAngle[2] &&
        motorposition[3] == motorStartAngle[3] && motorposition[4] == motorStartAngle[4]) {
      break;
    }
    delay(10);
  }
}

void hello() { // starting open hand
  openhand();
  for (int i = 1; i <= angleChange; i++) { // thumb moving 90
    motorposition[0]++;
    Servo1.write(motorposition[0]);
    delay(10);
  }
}

void handshake() { // starting open hand
  openhand();
  for (int i = 1; i <= angleChange; i++) {
    if (i % 2 == 0) { // thumb moving 45
      motorposition[0]++;
      Servo1.write(motorposition[0]);
    }
    if (i % 4 == 0) { // index moving 23
      motorposition[1]++;
      Servo2.write(motorposition[1]);
    }
    if (i % 4 == 0) { // middle moving 23
      motorposition[2]++;
      Servo3.write(motorposition[2]);
    }
    if (i % 3 == 0) { // ring finger moving 34
      motorposition[3]++;
      Servo4.write(motorposition[3]);
    }
    if (i % 2 == 0) { // pinky moving 45
      motorposition[4]++;
      Servo5.write(motorposition[4]);
    }
    delay(7);
  }
}

void no() {
  openhand();
  for (int i = 1; i <= angleChange; i++) {
      motorposition[0]++;
      Servo1.write(motorposition[0]);
    if (i % 2 == 0) { // index moving 45
      motorposition[1]++;
      Servo2.write(motorposition[1]);
    }
    if (i % 2 == 0) { // middle moving 45
      motorposition[2]++;
      Servo3.write(motorposition[2]);
    }
    // ring finger moving 80
    motorposition[3]++;
    Servo4.write(motorposition[3]);
    // pinky moving 80
    motorposition[4]++;
    Servo5.write(motorposition[4]);
    delay(10);
  }
}

void goodbye() { // starting openhand
  for (int i = 1; i <= 3; i++) {
    openhand();
    for (int i = 1; i <= angleChange; i++) {
      motorposition[1]++;
      Servo2.write(motorposition[1]);
      motorposition[2]++;
      Servo3.write(motorposition[2]);
      motorposition[3]++;
      Servo4.write(motorposition[3]);
      motorposition[4]++;
      Servo5.write(motorposition[4]);
      delay(10);
    }
  }
}

void switchstatement(String data) {
  data.trim(); // Remove any leading or trailing whitespace
  if (data == "hello") {
    hello();
  } else if (data == "no") {
    no();
  } else if (data == "thankyou") {
    openhand();
  } else if (data == "highfive") {
    openhand();
  } else if (data == "handshake") {
    handshake();
  } else if (data == "goodbye") {
    goodbye();
  } else if (data == "yes") {
    closehand();
  } else if (data == "fistbump") {
    closehand();
  } else if (data == "openhand") {
    openhand();
    delay(500);
  } else {
    Serial.println("did not execute");
  }

}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readString();
    Serial.println("You sent me: " + data);
    switchstatement(data);
  }
}
