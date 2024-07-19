#include <Arduino.h>

#define rotaryPin1 25
#define rotaryPin2 26

volatile int clickChange, clickCounter = 0, prev_clickChange = 0;

void clickCount();

void setup() {
  Serial.begin(115200);
  pinMode(rotaryPin1, INPUT_PULLUP);
  pinMode(rotaryPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rotaryPin1), clickCount, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin2), clickCount, CHANGE);
}

void loop() {
  int prev_clickCounter;
  if(clickCounter != prev_clickCounter){
  Serial.println(clickCounter);
  delay(100);
  }
  prev_clickCounter = clickCounter;
}

void clickCount(){
  clickChange = 2 * digitalRead(rotaryPin1) + 1 * digitalRead(rotaryPin2);
  if(clickChange == 1 && prev_clickChange == 3 || clickChange == 0 && prev_clickChange == 1 ||
   clickChange == 2 && prev_clickChange == 0 || clickChange == 3 && prev_clickChange == 2){
    clickCounter--;
   } else if (clickChange == 3 && prev_clickChange == 1 || clickChange == 2 && prev_clickChange == 3 ||
   clickChange == 0 && prev_clickChange == 2 || clickChange == 1 && prev_clickChange == 0){
    clickCounter++;
   }
  prev_clickChange = clickChange;

}
