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

  xTaskCreate(toggleLED, "toggleLED", 2048, NULL, 1, &LEDtaskHandle);
}

void loop() {
  LED1Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED2Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED3Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED4Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED5Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED6Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED7Flag = true;
  vTaskDelay(1000/portTICK_PERIOD_MS);
  LED8Flag = true;

}

void toggleLED(void *params){
   while(1) { 
      if (LED1Flag) {
        changeMUX(0, 0, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED2Flag) {
        changeMUX(0, 1, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED3Flag) {
        changeMUX(0, 0, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED4Flag) {
        changeMUX(0, 1, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED5Flag) {
        changeMUX(1, 0, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED6Flag) {
        changeMUX(1, 0, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED7Flag) {
        changeMUX(1, 1, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (LED8Flag) {
        changeMUX(1, 1, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
   }
}

void changeMUX(bool S0, bool S1, bool S2) {
  digitalWrite(MUX1, S0);
  digitalWrite(MUX2, S1);
  digitalWrite(MUX3, S2); 
}