#include <Arduino.h>

#define LED 32 //pin labelled 32 on the board

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  Serial.println("on");
  delay(1000);
  digitalWrite(LED, LOW);
  Serial.println("off");
  delay(1000);
}