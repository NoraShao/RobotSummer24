#include <Arduino.h>
#define IR1 0
#define IR2 4
#define IR3 37
#define IR4 38

void setup() {
  Serial.begin(115200);
  pinMode(IR1, INPUT_PULLDOWN);
  pinMode(IR2, INPUT_PULLDOWN);
  pinMode(IR3, INPUT_PULLDOWN);
  pinMode(IR4, INPUT_PULLDOWN);
}

void loop() {
  Serial.println(digitalRead(IR1));
  Serial.println(digitalRead(IR2));
  Serial.println(digitalRead(IR3));
  Serial.println(digitalRead(IR4));
  Serial.println();
  delay(100);
}
