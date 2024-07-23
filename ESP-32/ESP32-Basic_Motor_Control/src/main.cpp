#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define PWM_INPUT 8
#define PWM_OUTPUT 13
#define PWM_FREQ 100
#define PWM_RESOLUTION 12
#define PWM_CHANNEL 13

volatile int input;


void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.println("Basic motor \ncontrol");
  display_handler.display();
  delay(1000);
  display_handler.clearDisplay();
  display_handler.display();

  pinMode(PWM_INPUT, INPUT);
  pinMode(PWM_OUTPUT, OUTPUT);

  // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(PWM_OUTPUT, PWM_CHANNEL);
}

void loop() {
  input = (analogRead(PWM_INPUT) / ((double) 1023)) * 4096;
  display_handler.clearDisplay();
  display_handler.setCursor(1,1);
  display_handler.println("Input:");
  display_handler.println(input);
  display_handler.display();
  //pwm_start(IO13, PWM_FREQ, input, RESOLUTION_12B_COMPARE_FORMAT);
  ledcWrite(PWM_OUTPUT, input);
}