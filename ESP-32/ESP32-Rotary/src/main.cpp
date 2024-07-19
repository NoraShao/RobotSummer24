#include <Arduino.h>
#include <ESP32Encoder.h>

#define rotaryPin1 25
#define rotaryPin2 26

ESP32Encoder encoder;
int position = 0;
int previous;

void setup() {
  Serial.begin(115200);
  pinMode(rotaryPin1, INPUT);
  pinMode(rotaryPin2, INPUT);
  encoder.attachSingleEdge(rotaryPin1, rotaryPin2);
  encoder.setCount(0);
  Serial.println(encoder.getCount());
}

void loop() {
  previous = position;
  position = encoder.getCount();
  // Serial.println(position);
  // delay(500);
  if (previous != position) {
    Serial.println(position);
  }
}