#include <AFMotor.h>
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);



void setup() {
  motor1.setSpeed(100);
  motor2.setSpeed(255);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  Serial.begin(9600);
   
}


void loop() {
  if (Serial.available()) {
    char  recvd = Serial.read();
    if (recvd == 'q') {
      motor1.run(FORWARD);
      delay(250);
      motor1.run(RELEASE);
      Serial.print("ok");
    }
    if (recvd == 'w') {
      motor1.run(BACKWARD);
      delay(250);
      motor1.run(RELEASE);


    }
    if (recvd == 'e') {
      motor2.run(FORWARD);
      delay(200);
      motor2.run(RELEASE);
    }
    if (recvd == 'r') {
      motor2.run(BACKWARD);
      delay(200);
      motor2.run(RELEASE);
    }
    if (recvd == 't') {
      motor3.run(FORWARD);
      delay(250);
      motor3.run(RELEASE);
    }
    if (recvd == 'y') {
      motor3.run(BACKWARD);
      delay(250);
      motor3.run(RELEASE);
    }
    if (recvd == 'u') {
      motor4.run(FORWARD);
      delay(250);
      motor4.run(RELEASE);
    }
    if (recvd == 'i') {
      motor4.run(BACKWARD);
      delay(250);
      motor4.run(RELEASE);
    }
  }
}
