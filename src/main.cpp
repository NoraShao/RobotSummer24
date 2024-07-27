#include <Arduino.h>

#define MUX1 25
#define MUX2 26
#define MUX3 32

bool LED1Flag = false;
bool LED2Flag = false;
bool LED3Flag = false;
bool LED4Flag = false;
bool LED5Flag = false;
bool LED6Flag = false;
bool LED7Flag = false;
bool LED8Flag = false;

void LEDSwitch();
void toggleLED(void *params);
void changeMUX(bool S0, bool S1, bool S2);

TaskHandle_t LEDtaskHandle;

void setup() {

  Serial.begin(115200);
  pinMode(MUX1, OUTPUT);
  pinMode(MUX2, OUTPUT);
  pinMode(MUX3, OUTPUT);

  LED1Flag = true;
  LED2Flag = true;
  LED3Flag = true;
  LED4Flag = true;
  LED5Flag = true;
  LED6Flag = true;
  LED7Flag = true;
  LED8Flag = true;

  xTaskCreate(toggleLED, "toggleLED", 4096, NULL, 1, &LEDtaskHandle);
}

void loop() {
}

void LEDSwitch(){
  
}

void toggleLED(void *params){
   while(1) { 
      if (LED1Flag) {
        changeMUX(0, 0, 0);
        delay(1);
      }
      if (LED2Flag) {
        changeMUX(0, 0, 1);
        delay(1);
      }
      if (LED3Flag) {
        changeMUX(0, 1, 0);
        delay(1);
      }
      if (LED4Flag) {
        changeMUX(0, 1, 1);
        delay(1);
      }
      if (LED5Flag) {
        changeMUX(1, 0, 0);
        delay(1);
      }
      if (LED6Flag) {
        changeMUX(1, 0, 1);
        delay(1);
      }
      if (LED7Flag) {
        changeMUX(1, 1, 0);
        delay(1);
      }
      if (LED8Flag) {
        changeMUX(1, 1, 1);
        delay(1);
      }
   }
}

void changeMUX(bool S0, bool S1, bool S2) {
  digitalWrite(MUX1, S0);
  digitalWrite(MUX2, S1);
  digitalWrite(MUX3, S2); 
}