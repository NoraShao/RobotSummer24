#include <Arduino.h>

#define IN1 39
#define IN2 36

const int CH1 = 0;
const int CH2 = 1;
const int PWMRes = 12;
const int PWMFreq = 100;

void setSpeed(char motorDirection, int speed);
//sets speed of motor. use either 'f' for forwards or 'b' for backwards. speed can be set between 0 and 4096

void setup() {
  Serial.begin(115200);
  ledcSetup(CH1, PWMFreq, PWMRes);
  ledcSetup(CH2, PWMFreq, PWMRes);
  ledcAttachPin(IN1, CH1);
  ledcAttachPin(IN2, CH2);
}

void loop() {
  //this stuff can be replaced to do whatever you need it to
  setSpeed('f', 4096);
  delay(5000);
  setSpeed('f', 0);
  delay(5000);
  setSpeed('b', 4096);
}

void setSpeed(char motorDirection, int speed){
  if(motorDirection == 'f'){
      ledcWrite(CH1, speed);
      ledcWrite(CH2, 0);
    } else if (motorDirection == 'b'){
      ledcWrite(CH1, 0);
      ledcWrite(CH2, speed);
    } else {
      Serial.println("set speed fail");
    }
}
