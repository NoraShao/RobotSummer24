#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

//variables
#define start_pin 37
#define IR_sideRight 5
#define IR_sideLeft 10
#define IR_farRight 22
#define IR_right 19
#define IR_left 8
#define IR_farLeft 7
#define IN1 15
#define IN2 13
#define IN3 21
#define IN4 20
#define LED1 25
#define LED2 26
#define LED3 32
#define LED4 33
/*
#define rotaryPin1 5
#define rotaryPin2 10
#define rotaryPin3 4
#define rotaryPin4 2
*/
#define microswitch 9
#define servoIN 27
#define clawIN1 12
#define clawIN2 14

const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 2;
const int CH4 = 3;
const int clawCH1 = 0;
const int clawCH2 = 1;
const int PWMRes = 12;
const int PWMFreq = 100;

const double set_speed = 1500;
double other_speed = 1500;
const int P = 30;
const int I = 0;
const int D = 0.5;
const int rP = 0;
const int rI = 0;
const int rD = 0;

const int serve_area_far = 0;
const int serve_area_close = 0;

const int updownSpeed = 2048;
const int homeAngle = 80;
volatile int currentAngle = homeAngle;
const unsigned long upTime = 2000;

volatile int clickL;
volatile int clickCounterL;
volatile int prev_clickL;
volatile int clickR;
volatile int clickCounterR;
volatile int prev_clickR;
volatile int IRCounter = 0;
volatile int position_difference = 100;
volatile bool lineFollowFlag = true;
volatile bool backFlag = false;

TaskHandle_t balanceTaskHandle;
Servo myServo;

//functions

void startUp();
//starts main loop when start_pin goes high

void shutDown();
//permanetly stops loop until reset

void stop();
//stops the driver wheels

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right);
//sets speed and direction for driver wheels

void turn(char direction);
//turns the driver wheels

void linefollow(char motor_direction);
//follows line for the provided time

void linefollowtime(unsigned long time);

void goTo(int initialPosition, int finalPosition);
//goes from one line to another

void upTo(int upTo_delay);
//moves up to counter

void backUp();
//goes back to main black line

void locateServeArea(unsigned long timer);
//void locateServeArea(int currentPosition, char motor_direction);
//goes to horizontal position of serving area

double getError();
//locates position of IRs on the black tape

void grab(String food);
//grabs food or plate

void stack(String food);
//stacks held item on plate

void cook();
//holds food on stove for 10 seconds

void resetRotaryCount();
//resets rotary count

void clickCountLeft();
//counts rotary clicks on the left wheel

void clickCountRight();
//counts rotary clicks on the right wheel

void balanceMotors(void *param);
//controls the right motor to go the same speed as the left

void startBalance();
//starts balancing motor speed

void stopBalance();
//stops balancing motor speed

void IRCount();
//counts number of activations from wing IR sensors

void LEDSwitch();
//Toggles LED deoending on state of IR sensor

//set up

void setup() {
  
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(IR_farRight), LEDSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_right), LEDSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_left), LEDSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_farLeft), LEDSwitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_sideLeft), IRCount, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_sideRight), IRCount, CHANGE);
  
  /*
  attachInterrupt(digitalPinToInterrupt(rotaryPin1), clickCountLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin2), clickCountLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin3), clickCountRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin4), clickCountRight, CHANGE);
  */
  
  //input
  pinMode(start_pin, INPUT);
  pinMode(IR_sideRight, INPUT_PULLUP);
  pinMode(IR_sideLeft, INPUT_PULLUP);
  pinMode(IR_farRight, INPUT_PULLUP);
  pinMode(IR_right, INPUT_PULLUP);
  pinMode(IR_left, INPUT_PULLUP);
  pinMode(IR_farLeft, INPUT_PULLUP);
  /*
  pinMode(rotaryPin1, INPUT_PULLUP);
  pinMode(rotaryPin2, INPUT_PULLUP);
  pinMode(rotaryPin3, INPUT_PULLUP);
  pinMode(rotaryPin4, INPUT_PULLUP);
  */
  pinMode(microswitch, INPUT_PULLDOWN);

  //output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  ledcSetup(CH1, PWMFreq, PWMRes);
  ledcSetup(CH2, PWMFreq, PWMRes);
  ledcSetup(CH3, PWMFreq, PWMRes);
  ledcSetup(CH4, PWMFreq, PWMRes);
  ledcSetup(clawCH1, PWMFreq, PWMRes);
  ledcSetup(clawCH2, PWMFreq, PWMRes);
  ledcAttachPin(IN1, CH1);
  ledcAttachPin(IN2, CH2);
  ledcAttachPin(IN3, CH3);
  ledcAttachPin(IN4, CH4);
  ledcAttachPin(clawIN1, clawCH1);
  ledcAttachPin(clawIN2, clawCH2);
  myServo.attach(servoIN);
  myServo.write(homeAngle);
}

//loop

void loop() {
  goTo(0,2);
  turn('r');
  upTo(500);
  grab("cheese");
  backUp();
  turn('l');
  goTo(2,6);
  turn('l');
  upTo(500);
  stack("cheese");
  grab("plate");
  backUp();
  turn('r');
  locateServeArea(5000);
  turn('r');
  upTo(200);
  stack("plate");
  shutDown();
}

//function definitions

void startUp() {
  while(digitalRead(start_pin) == LOW){}
}

void shutDown(){
  stop();
  while(1){}
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
  }
  if(right_motor == 'f'){
    ledcWrite(CH3, speed_right);
    ledcWrite(CH4, 0);
  } else if (right_motor == 'b'){
    ledcWrite(CH3, 0);
    ledcWrite(CH4, speed_right);
  }
}

void turn(char direction){
  /*
  startBalance();
  while(clickCounterL <= 24){
    if(direction == 'l'){
    setSpeed('b', 'f', set_speed, other_speed);
    } else if(direction == 'r'){
    setSpeed('f', 'b', set_speed, other_speed);
    }
  }
  stopBalance();
  */
  if(direction == 'r'){
    setSpeed('b', 'f', set_speed, set_speed);
    } else if(direction == 'l'){
    setSpeed('f', 'b', set_speed, set_speed);
    }
    //
  delay(369);
  stop();
}

void linefollow(char motorDirection){
  /*
  unsigned long previous_time = millis(), current_time;
  double previous_error = 0, error_sum = 0, error_difference = 0, error, PID;
  //startBalance();
  while(lineFollowFlag){
    error = getError();
    while(error == -0.001){
      backFlag = true;
      setSpeed('b', 'b', set_speed, set_speed );
      delay(500);
      error = getError();
    }
    backFlag = false;
    current_time = millis();
    error_sum += error * (current_time - previous_time);
    error_difference = current_time - previous_time != 0 ? (error - previous_error) / (current_time - previous_time) : 0;
    PID = P * error + I * error_sum + D * error_difference;
    previous_error = error;
    previous_time = current_time;
    if(motorDirection == 'f'){
      setSpeed('f', 'f', set_speed + PID, set_speed - PID);
    } else if(motorDirection == 'b'){
      setSpeed('b', 'b', set_speed - PID, set_speed + PID);
    }
    delay(10);
    /*
    Serial.println(clickCounterL);
    Serial.println(clickCounterR);
    delay(250);
    Serial.println();
  }
  //stopBalance();
  */
 while(lineFollowFlag){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 8: setSpeed('f','f', set_speed, 0); break;
    case 12: setSpeed('f','f', set_speed, set_speed * 0.5); break;
    case 14: setSpeed('f','f', set_speed, set_speed * 0.5); break;
    case 4: setSpeed('f','f', set_speed, set_speed * 0.9); break;
    case 6: setSpeed('f','f', set_speed, set_speed); break;
    case 2: setSpeed('f','f', set_speed * 0.9, set_speed); break;
    case 7: setSpeed('f','f', set_speed * 0.5, set_speed); break;
    case 3: setSpeed('f','f', set_speed * 0.5, set_speed); break;
    case 1: setSpeed('f','f', 0, set_speed); break;
    case 15: setSpeed('f','f', set_speed, set_speed); break;
    default: {
      Serial.println("getError fail");
      setSpeed('f','f', set_speed, 0);
    }
  }
 }
}

void linefollowtime(unsigned long time){
  unsigned long initial_time = millis(), new_time = millis();
  while(time < new_time - initial_time){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 8: setSpeed('f','f', set_speed, 0); break;
    case 12: setSpeed('f','f', set_speed, set_speed * 0.5); break;
    case 14: setSpeed('f','f', set_speed, set_speed * 0.5); break;
    case 4: setSpeed('f','f', set_speed, set_speed * 0.9); break;
    case 6: setSpeed('f','f', set_speed, set_speed); break;
    case 2: setSpeed('f','f', set_speed * 0.9, set_speed); break;
    case 7: setSpeed('f','f', set_speed * 0.5, set_speed); break;
    case 3: setSpeed('f','f', set_speed * 0.5, set_speed); break;
    case 1: setSpeed('f','f', 0, set_speed); break;
    case 15: setSpeed('f','f', set_speed, set_speed); break;
    default: {
      Serial.println("getError fail");
      setSpeed('f','f', set_speed, 0);
    }
  }
  new_time = millis();
 }
}

void goTo(int initialPosition, int finalPosition){
  IRCounter = 0;
  position_difference = abs(finalPosition - initialPosition);

  char motorDirection = initialPosition < finalPosition ? 'f' : 'b';

  linefollow(motorDirection);

  lineFollowFlag = true;
  setSpeed('b','b',1500,1500);
  delay(200);
  stop();
}

void upTo(int upToClicks){
  setSpeed('f','f',set_speed, set_speed);
  delay(upToClicks);
  stop();
  /*
  while(clickCounterL <= upToClicks){
    linefollow('f');
  }
  stop();
  */
}

void backUp(){
  /*
  double error = getError();
  while(error != -0.001){
    linefollow('b');
    error = getError();
  }
  while(error == -0.001){
    startBalance();
    setSpeed('b','b', set_speed, other_speed);
    error = getError();
  }
  //stopBalance();
  stop();
  */
  while(digitalRead(IR_left) == LOW){
   setSpeed('b','b', set_speed,set_speed); 
  }
  stop();
}

void locateServeArea(unsigned long timer){
  unsigned long initial_time = millis(), new_time = millis();
  while(timer < new_time - initial_time){
    linefollow('f');
    new_time = millis();
  }

  /*
  int distance_to_area;
  if(currentPosition == 1 || currentPosition == 6){
    distance_to_area = serve_area_far;
  } else if(currentPosition == 4){
    distance_to_area = serve_area_close;
  }
  startBalance();
  while(clickCounterL <= distance_to_area){
    setSpeed(motorDirection, motorDirection, set_speed, other_speed);
  }
  stopBalance();
  stop();
  */
}

double getError(){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 0: return -0.001; break;
    case 8: return 3; break;
    case 12: return 2.5; break;
    case 14: return 2; break;
    case 4: return 1; break;
    case 6: return 0; break;
    case 2: return -1; break;
    case 7: return -2; break;
    case 3: return -2.5; break;
    case 1: return -3; break;
    case 15: return 0.001; break;
    default: {
      Serial.println("getError fail");
      return 0;
    }
  }
}

void grab(String food){
  int finalAngle;
  if(food == "lectuce"){
    finalAngle = 128;
  } else if (food == "tomato"){
    finalAngle = 124;
  } else if (food == "patty"){
    finalAngle = 124;
  } else if (food == "bun"){
    finalAngle = 123;
  } else if (food == "cheese"){
    finalAngle = 126; 
  } else if (food == "plate"){
    finalAngle = 106;
  }
  for(int i = currentAngle; i >= homeAngle; i--){
    myServo.write(i);
    delay(10);
  }
  currentAngle = homeAngle;
  while(!digitalRead(microswitch)){
    ledcWrite(clawCH1, 0);
    ledcWrite(clawCH2, updownSpeed);
    delay(100);
  }
  ledcWrite(clawCH2, 0);
  for(int i = currentAngle; i <= finalAngle; i++){
    myServo.write(i);
    delay(10);
  }
  currentAngle = finalAngle;
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  delay(upTime);
  ledcWrite(clawCH1, 0);
}

void stack(String food){
  ledcWrite(clawCH1, 0);
  ledcWrite(clawCH2, updownSpeed);
  if(food == "lectuce"){
    delay(700);  
  } else if (food == "tomato"){
    delay(800);
  } else if (food == "patty"){
    delay(900);
  } else if (food == "topBun"){
    delay(500);
  } else if (food == "bottomBun"){
    delay(1000);
  } else if (food == "cheese"){
    delay(1000);
  } else if (food == "plate"){
    delay(1500);
  }
  ledcWrite(clawCH2, 0);
  for(int i = currentAngle; i >= homeAngle; i--){
    myServo.write(i);
    delay(10);
  }
  currentAngle = homeAngle;
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  delay(upTime);
  ledcWrite(clawCH1, 0);
}

void cook(){
  while(!digitalRead(microswitch)){
    ledcWrite(clawCH1, 0);
    ledcWrite(clawCH2, updownSpeed);
    delay(100);
  }
  ledcWrite(clawCH2, 0);
  delay(10000);
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  delay(upTime);
  ledcWrite(clawCH1, 0);
}

void resetRotaryCount(){
  clickCounterL = 0;
  prev_clickL = 0;
  clickCounterR = 0;
  prev_clickR = 0;
}

void clickCountLeft(){
  /*
  clickL = 2 * digitalRead(rotaryPin1) + 1 * digitalRead(rotaryPin2);
  if(clickL == 1 && prev_clickL == 3 || clickL == 0 && prev_clickL == 1 ||
   clickL == 2 && prev_clickL == 0 || clickL == 3 && prev_clickL == 2){
    clickCounterL--;
   } else if (clickL == 3 && prev_clickL == 1 || clickL == 2 && prev_clickL == 3 ||
   clickL == 0 && prev_clickL == 2 || clickL == 1 && prev_clickL == 0){
    clickCounterL++;
   }
  prev_clickL = clickL;
  */

}

void clickCountRight(){
  /*
  clickR = 2 * digitalRead(rotaryPin3) + 1 * digitalRead(rotaryPin4);
  if(clickR == 1 && prev_clickR == 3 || clickR == 0 && prev_clickR == 1 ||
   clickR == 2 && prev_clickR == 0 || clickR == 3 && prev_clickR == 2){
    clickCounterR--;
   } else if (clickR == 3 && prev_clickR == 1 || clickR == 2 && prev_clickR == 3 ||
   clickR == 0 && prev_clickR == 2 || clickR == 1 && prev_clickR == 0){
    clickCounterR++;
   }
  prev_clickR = clickR;
  */
}

void balanceMotors(void *param){
  int prevRotaryError = 0;
  unsigned long prevRotaryTime = millis();
  int prevClickCounterL = 0;
  int prevClickCounterR = 0;
  resetRotaryCount();
  while(1){
    int clickChangeL = clickCounterL - prevClickCounterL;
    int clickChangeR = clickCounterR - prevClickCounterR;
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - prevRotaryTime;
    if(timeElapsed == 0){
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    double speedL = (double) clickChangeL / timeElapsed;
    double speedR = (double) clickChangeR / timeElapsed;
    double error = speedR - speedL;
    double error_sum = error_sum + error * (timeElapsed);
    double error_difference = (error - prevRotaryError) / (timeElapsed);
    double PID = rP * error + rI * error_sum + rD * error_difference;
    prevRotaryError = error;
    prevRotaryTime = currentTime;
    other_speed = set_speed - PID;
    prevClickCounterL = clickCounterL;
    prevClickCounterR = clickCounterR;
    /*
    Serial.println(speedL);
    Serial.println(speedR);
    Serial.println();
    */
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void startBalance(){
  xTaskCreate(balanceMotors, "balanceMotors", 2048, NULL, 1, &balanceTaskHandle);
}

void stopBalance(){
  vTaskDelete(balanceTaskHandle);
  other_speed = set_speed;
}

void IRCount(){
  /*
  if(digitalRead(IR_sideLeft) == 1){
    digitalWrite(LED1, HIGH);
    IRCounter++;
  }
  if(digitalRead(IR_sideLeft) == 0){
    digitalWrite(LED1, LOW);
  }
  if(digitalRead(IR_sideRight) == 1){
    digitalWrite(LED4, HIGH);
    IRCounter++;
  }
  if(digitalRead(IR_sideRight) == 0){
    digitalWrite(LED4, LOW);
  }
  */
  if(backFlag == false && (digitalRead(IR_sideLeft) || digitalRead(IR_sideRight))){
    IRCounter++;
    if(IRCounter >= position_difference){
      lineFollowFlag = false;
    }
  }
}

void LEDSwitch(){
  if(digitalRead(IR_farRight) == 1){
    digitalWrite(LED1, HIGH);
  }
  if(digitalRead(IR_right) == 1){
    digitalWrite(LED2, HIGH);
  }
  if(digitalRead(IR_left) == 1){
    digitalWrite(LED3, HIGH);
  }
  if(digitalRead(IR_farLeft) == 1){
    digitalWrite(LED4, HIGH);
  }
  if(digitalRead(IR_farRight) == 0){
    digitalWrite(LED1, LOW);
  }
  if(digitalRead(IR_right) == 0){
    digitalWrite(LED2, LOW);
  }
  if(digitalRead(IR_left) == 0){
    digitalWrite(LED3, LOW);
  }
  if(digitalRead(IR_farLeft) == 0){
    digitalWrite(LED4, LOW);
  }
}