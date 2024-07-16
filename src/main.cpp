#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
/*
#include <esp_now.h>
#include <WiFi.h>
*/

//variables
#define start_pin 25
#define IR_frontleft 26
#define IR_frontright 32
#define IR_backleft 33
#define IR_backright 27
#define IR_middleleft 14
#define IR_middle 12
#define IR_middleright 13
#define IN1 4
#define IN2 0
#define IN3 15
#define IN4 2

const int CH1 = 0;
const int CH2 = 1;
const int CH3 = 2;
const int CH4 = 3;
const int PWMRes = 12;
const int PWMFreq = 100;

const int set_speed = 2048;
const int turn_speed = 2048;
const int P = 0;
const int I = 0;
const int D = 0;

const int turn90_delay = 0;
const int offsetTimer = 0;
const int upTo_timer = 0;
const int far_timer = 0;
const int close_timer = 0;

/*
bool go;

uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9d, 0x66, 0x50}; //robot 1 recieving
//uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9c, 0x61, 0x44}; //robot 2 recieving
esp_now_peer_info_t peerInfo;

typedef struct flag{
  bool act;
} flag;

flag trigAction;
*/
//functions

void startUp();
//starts main loop when start_pin goes high

void shutDown();
//

void stop();
//stops the driver wheels

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right);
//sets speed and direction for driver wheels

void turn(char direction, int turn_delay);
//turns the driver wheels

void linefollowTimer(unsigned long time_interval, char motor_direction);
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

/*
void sendFlag();
//sends flag to other robot

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status);
//confirms sendFlag

void receiveFlag();
//holds action until flag is received

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int message_length);
//confirms flag received
*/

//set up

void setup() {
  
  Serial.begin(115200);

  //communication
  /*
  WiFi.mode(WIFI_STA);
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
  */
  
  //input
  pinMode(start_pin, INPUT);
  pinMode(IR_frontright, INPUT);
  pinMode(IR_frontleft, INPUT);
  pinMode(IR_backright, INPUT);
  pinMode(IR_backleft, INPUT);
  pinMode(IR_middleleft, INPUT);
  pinMode(IR_middle, INPUT);
  pinMode(IR_middleright, INPUT);
  
  //output
  ledcSetup(CH1, PWMFreq, PWMRes);
  ledcSetup(CH2, PWMFreq, PWMRes);
  ledcSetup(CH3, PWMFreq, PWMRes);
  ledcSetup(CH4, PWMFreq, PWMRes);
  ledcAttachPin(IN1, CH1);
  ledcAttachPin(IN2, CH2);
  ledcAttachPin(IN3, CH3);
  ledcAttachPin(IN4, CH4);
}

//loop

void loop() {
  ledcWrite(CH1, 4095);
  ledcWrite(CH2, 0);
  ledcWrite(CH3, 4095);
  ledcWrite(CH4, 0);
  delay(4000);
  stop();
  delay(4000);
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 4095);
  ledcWrite(CH3, 0);
  ledcWrite(CH4, 4095);
  delay(4000);
  stop();
  delay(4000);
 /*
 Serial.println(digitalRead(IR_middleleft));
 Serial.println(digitalRead(IR_middle));
 Serial.println(digitalRead(IR_middleright));
 Serial.println();
 delay(1000);
 */
}

//function definitions

void startUp() {
  while(digitalRead(start_pin) == LOW){
  }
}

void shutDown(){
  stop();
  while(1){
    delay(1000);
  }
}

void stop(){
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
  ledcWrite(CH3, 0);
  ledcWrite(CH4, 0);
}

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right){
  if(left_motor == 'f'){
      ledcWrite(CH1, speed_left);
      ledcWrite(CH2, 0);
    } else if (left_motor == 'b'){
      ledcWrite(CH1, 0);
      ledcWrite(CH2, speed_left);
    } else {
      Serial.println("set speed_left fail");
    }

    if(right_motor == 'f'){
      ledcWrite(CH3, speed_right);
      ledcWrite(CH4, 0);
    } else if (right_motor == 'b'){
      ledcWrite(CH3, 0);
      ledcWrite(CH4, speed_right);
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
  double previous_error = 0, error_sum = 0, error_difference = 0, error, PID;
  while(new_time < initial_time + time_interval){
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    error_difference = (error - previous_error) / (new_time - initial_time);
    PID = P * error + I * error_sum + D * error_difference;
    setSpeed(motorDirection, motorDirection, set_speed + PID, set_speed - PID);
    previous_error = error;
  }
}

void goTo(int initialPosition, int finalPosition){

  int difference = abs(finalPosition - initialPosition);
  int IR_left, IR_right, linesPassed = 0;
  char motorDirection;

  motorDirection = initialPosition < finalPosition ? 'f' : 'b';

  if(motorDirection == 'f'){
    IR_left = IR_frontleft;
    IR_right = IR_frontright; 
  } else if (motorDirection == 'b'){
    IR_left = IR_backleft;
    IR_right = IR_backright;
  } else {
    Serial.println("gotTo fail");
  }
  
  unsigned long initial_time = millis(), new_time;
  double previous_error = 0, error_sum = 0, error_difference = 0, error, PID;
  while(linesPassed < difference){
    if(digitalRead(IR_left) == HIGH || digitalRead(IR_right) == HIGH){
      linesPassed++;
    }
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    error_difference = (error - previous_error) / (new_time - initial_time);
    double PID = P * error + I * error_sum + D * error_difference;
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
  double previous_error = 0, error_sum = 0, error_difference = 0, error, PID;
  unsigned long initial_time = millis(), new_time;
  while(error != -7){
    error = getError();
    new_time = millis();
    error_sum += error * (new_time - initial_time);
    error_difference = (error - previous_error) / (new_time - initial_time);
    PID = P * error + I * error_sum + D * error_difference;
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
  int leftIR_reading = digitalRead(IR_middleleft);
  int middleIR_reading = digitalRead(IR_middle);
  int rightIR_reading = digitalRead(IR_middleright);
  int bitSum = 4 * leftIR_reading + 2 * middleIR_reading + 1 * rightIR_reading;
  switch(bitSum){
    case 4: return -2;
    case 6: return -1;
    case 2: return 0;
    case 3: return 1;
    case 1: return 2;
    case 7: return 0;
    case 0: return -7;
    default: {
      Serial.println("getError fail");
      return -99;
    }
  }
}

/*
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
*/