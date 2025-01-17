#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>

//variables

#define START_PIN 8
#define IR_FRONTLEFT 2
#define IR_FRONTRIGHT 8
#define IR_BACKLEFT 4
#define IR_BACKRIGHT 5
#define IR_MIDDLELEFT 8
#define IR_MIDDLE 7
#define IR_MIDDLERIGHT 8
#define IN1 20
#define IN2 21
#define IN3 22
#define IN4 19

const int set_speed = 512;
const int turn_speed = 512;
const int P = 0;
const int I = 0;
const int D = 0;

const int turn90_delay = 100;
const int offsetTimer = 100;
const int upTo_timer = 100;
const int far_timer = 100;
const int close_timer = 100;

// Set your Static IP address 192.168.109.118
IPAddress local_IP(192, 168, 109, 118); // need this (static IP address of ESP32)
// Set your Gateway IP address
IPAddress gateway(192, 168, 109, 80); // need this (can change with the router) 192.168.109.80

IPAddress subnet(255, 255, 255, 0); // need this

bool go;

//uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9d, 0x66, 0x50}; //robot 1 recieving
uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9c, 0x61, 0x44}; //robot 2 recieving
esp_now_peer_info_t peerInfo;

typedef struct flag{
  bool act;
} flag;

flag trigAction;

// gateway: 192.168.0.1
// duct ip 192.168.0.104
// scotch ip 192.168.0.105

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 105); // need this (static IP address of ESP32)
// Set your Gateway IP address
IPAddress gateway(192, 168,0, 1); // need this (can change with the router)

IPAddress subnet(255, 255, 0, 0); // need this

// const char* ssid = "TP-Link_C8D1"; // ssid of the wifi they're using
// const char* password = "93456593"; // wifi password
const char* ssid = "Jeff's iPhone"; // ssid of the wifi they're using
const char* password = "hissyandmineko";

//functions

void startUp();
//starts main loop when start_pin goes high

void stop();
//stops the driver wheels

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right);
//sets speed and direction for driver wheels

void turn(char direction, int turn_delay);
//turns the driver wheels

void linefollowTimer(int time_interval, char motor_direction);
//follows line for the provided time

void goTo(int initialPosition, int finalPosition);
//goes from one line to another

void upTo(int upTo_delay);
//moves up to counter

void backUp();
//goes back to main black line

void locateServeArea(int currentPosition, char motor_direction);
//goes to horizontal position of serving area

int getError();
//locates position of IRs on the black tape

void grab(String food);
//grabs food or plate

void stack(String food);
//stacks held item on plate

void cook();
//holds food on stove for 10 seconds

void sendFlag();
//sends flag to other robot

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status);
//confirms sendFlag

void receiveFlag();
//holds action until flag is received

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int message_length);
//confirms flag received

void setup() {
  Serial.begin(115200); // starts the serial monitor

  WiFi.mode(WIFI_STA); // need this to start the wifi

  // if (!WiFi.config(local_IP, gateway, subnet)) { // sets the ESP32 to a static IP address
  // Serial.println("STA Failed to configure");
  // }

  if (!WiFi.config(local_IP, gateway, subnet)) { // sets the ESP32 to a static IP address
  Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password); // joins wifi network
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) { // need this, but don't necessarily have to print to Serial
    delay(500);
    Serial.print(".");
  }

  // don't need this but it's useful information
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // OTA Configiration and Enable OTA
  Serial.println("\nEnabling OTA Feature");
  //ArduinoOTA.setPassword(""); // we can add a password if we want
  ArduinoOTA.begin();
  Serial.println("OTA started successfully :)");

  //input
  pinMode(START_PIN, INPUT);
  pinMode(IR_FRONTRIGHT, INPUT);
  pinMode(IR_FRONTLEFT, INPUT);
  pinMode(IR_BACKRIGHT, INPUT);
  pinMode(IR_BACKLEFT, INPUT);
  pinMode(IR_MIDDLELEFT, INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_MIDDLERIGHT, INPUT);
  //output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); //796288 //791680

  Serial.println("Initializing ESP-NOW");
  if(esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Error adding peer");
  }
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(esp_now_recv_cb_t(onReceive));
  Serial.println("ESP-NOW has started");
}

void loop()
{
  // OTA Handle
  ArduinoOTA.handle();
  stop();
}

//function definitions

void startUp() {
  while(START_PIN == LOW){
  }
}

void stop(){
  analogWrite(IN1, 0);
  analogWrite(IN2, 100);
  analogWrite(IN3, 0);
  digitalWrite(IN4, HIGH);
}

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right){
  if(left_motor == 'f'){
      analogWrite(IN1, speed_left);
      analogWrite(IN2, 0);
    } else if (left_motor == 'b'){
      analogWrite(IN1, 0);
      analogWrite(IN2, speed_left);
    } else {
      Serial.println("set speed_left fail");
    }

    if(right_motor == 'f'){
      analogWrite(IN3, speed_right);
      analogWrite(IN4, 0);
    } else if (right_motor == 'b'){
      analogWrite(IN3, 0);
      analogWrite(IN4, speed_right);
    } else {
      Serial.println("set speed_right fail");
    }
}

void turn(char direction, int turn_delay){
  if(direction == 'l'){
    setSpeed('b', 'f', turn_speed, turn_speed);
    delay(turn_delay);
  } else if(direction == 'r'){
    setSpeed('f', 'b', turn_speed, turn_speed);
    delay(turn_delay);
  } else {
    Serial.println("turn fail");
  }
  stop();
}

void linefollowTimer(unsigned long time_interval, char motorDirection){
  unsigned long initial_time = millis(), new_time;
  int previous_error, error_sum, error;
  while(new_time < initial_time + time_interval){
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    double PID = P * error + I * error_sum + D * (error - previous_error);
    setSpeed(motorDirection, motorDirection, set_speed + PID, set_speed - PID);
    previous_error = error;
  }
}

void goTo(int initialPosition, int finalPosition){

  int difference = abs(finalPosition - initialPosition);
  int IR_left, IR_right, linesPassed = 0, error, previous_error, error_sum;
  char motorDirection;

  motorDirection = initialPosition < finalPosition ? 'f' : 'b';

  if(motorDirection == 'f'){
    IR_left = IR_FRONTLEFT;
    IR_right = IR_FRONTRIGHT; 
  } else if (motorDirection == 'b'){
    IR_left = IR_BACKLEFT;
    IR_right = IR_BACKRIGHT;
  } else {
    Serial.println("gotTo fail");
  }
  
  unsigned long initial_time = millis(), new_time;
  while(linesPassed < difference){
    if(digitalRead(IR_left) == HIGH || digitalRead(IR_right) == HIGH){
      linesPassed++;
    }
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    double PID = P * error + I * error_sum + D * (error - previous_error);
    setSpeed(motorDirection, motorDirection, set_speed + PID, set_speed - PID);
    previous_error = error;
  }
  linefollowTimer(offsetTimer, motorDirection);
  stop();
}

void upTo(int upTo_time){
  linefollowTimer(upTo_time, 'f');
  stop();
}

void backUp(){
  int previous_error, error_sum, error;
  unsigned long initial_time = millis(), new_time;
  while(error != -7){
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    double PID = P * error + I * error_sum + D * (error - previous_error);
    setSpeed('b', 'b', set_speed + PID, set_speed - PID);
    previous_error = error;
  }
  while(error == -7){
    setSpeed('b','b', set_speed, set_speed);
  }
  stop();
}

void locateServeArea(int currentPosition, char motorDirection){
  int time_to_area;
  if(currentPosition == 1 || currentPosition == 6){
    time_to_area = far_timer;
  } else if(currentPosition == 4){
    time_to_area = close_timer;
  } else {
    Serial.println("locateServeArea fail");
  }
  linefollowTimer(time_to_area, motorDirection);
  stop();
}

int getError(){
  int leftIR_reading = digitalRead(IR_MIDDLELEFT);
  int middleIR_reading = digitalRead(IR_MIDDLE);
  int rightIR_reading = digitalRead(IR_MIDDLERIGHT);
  int sum = 4 * leftIR_reading + 2 * middleIR_reading + 1 * rightIR_reading;
  switch(sum){
    case 4: return -2;
    case 6: return -1;
    case 2: return 0;
    case 3: return 1;
    case 1: return 2;
    case 7: return 7;
    case 0: return -7;
    default: {
      Serial.println("getError fail");
      return -99;
    }
  }
}

void sendFlag(){
  trigAction.act = true;
  esp_err_t result = esp_now_send(MAC, (uint8_t *) &trigAction, sizeof(trigAction));
  if(result == ESP_OK){
    Serial.println("sent");
  } else {
    Serial.println("sendFlag fail");
  }
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void receiveFlag(){
  go = false;
  while(!go){
  }
}

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int message_length){
  memcpy(&trigAction, incomingData, sizeof(trigAction));
  go = true;
}