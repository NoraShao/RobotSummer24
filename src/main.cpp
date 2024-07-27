#include <Arduino.h>
#define MUX1 25
#define MUX2 26
#define MUX3 32

void setup() {
  pinMode(MUX1, OUTPUT);
  pinMode(MUX2, OUTPUT);
  pinMode(MUX3, OUTPUT);
  digitalWrite(MUX1, LOW);
  digitalWrite(MUX2, LOW);
  digitalWrite(MUX3, LOW);
}

void loop() {

}
