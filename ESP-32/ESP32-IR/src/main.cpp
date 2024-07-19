#include <Arduino.h>

#define farLeft_IR 4
#define left_IR 2
#define right_IR 37
#define farRight_IR 34
#define pin1 8
#define pin2 7
#define pin3 22
#define pin4 19

void setup() {
  Serial.begin(115200);
  pinMode(farLeft_IR, INPUT_PULLUP);
  pinMode(left_IR, INPUT_PULLUP);
  pinMode(right_IR, INPUT_PULLUP);
  pinMode(farRight_IR, INPUT_PULLUP);
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);

}

void loop() {
  // Serial.print(digitalRead(farLeft_IR));
  // Serial.print(digitalRead(left_IR));
  // Serial.print(digitalRead(right_IR));
  // Serial.println(digitalRead(farRight_IR));
  Serial.print(digitalRead(pin1));
  Serial.print(digitalRead(pin2));
  Serial.println(digitalRead(pin3));
  Serial.print(digitalRead(pin4));
  Serial.println();
  delay(100);
}