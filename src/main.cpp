#include <Arduino.h>

#define farLeft_IR 0
#define left_IR 4
#define right_IR 2
#define farRight_IR 15

void setup() {
  Serial.begin(115200);
  pinMode(farLeft_IR, INPUT);
  pinMode(left_IR, INPUT);
  pinMode(right_IR, INPUT);
  pinMode(farRight_IR, INPUT);
}

void loop() {
  Serial.println(digitalRead(farLeft_IR));
  Serial.println(digitalRead(left_IR));
  Serial.println(digitalRead(right_IR));
  Serial.println(digitalRead(farRight_IR));
  Serial.println();
  delay(100);
}
