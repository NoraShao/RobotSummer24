#include <Arduino.h>
#include <ESP32Servo.h>

Servo contServo;
const int servoPin = 7;
const int stopPW = 1500;
const int CWPW = 1000;
const int CCWPW = 2000;

void setup() {
  contServo.attach(servoPin);

}

void loop() {
  contServo.writeMicroseconds(CWPW);
  delay(5000);
  contServo.writeMicroseconds(CCWPW);
  delay(5000);
  contServo.writeMicroseconds(stopPW);
  delay(5000);
}
