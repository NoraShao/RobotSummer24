#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

//variables
#define IR_sideRight 22
#define IR_farRight 19
#define IR_right 8
#define IR_left 7
#define IR_farLeft 5
#define IR_sideLeft 10
#define IN1 13
#define IN2 15
#define IN3 21
#define IN4 20
#define MUX1 25
#define MUX2 26
#define MUX3 32
#define rotaryPin1 38
#define rotaryPin2 37
#define rotaryPin3 35
#define rotaryPin4 34
#define microswitch 9
#define limitSwitch 4
#define clawMovePin 2
#define servo1IN 27
#define servo2IN 33
#define clawIN1 14
#define clawIN2 12

//PWM setup
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 2;
const int CH4 = 3;
const int clawCH1 = 1;
const int clawCH2 = 6;
const int PWMRes = 12;
const int PWMFreq = 100;

//speed control
const double set_speed = 1200;
const double turn_speed = 1000;

//locating serving area
const int serve_area_far = 4000;
const int serve_area_close = 0;

//turning
unsigned long prevTime = millis();

//claw
const int homeAngle = 50;
const int lettuceAngle = 98;
const int tomatoAngle = 96;
const int cheeseAngle = 100;
const int pattyAngle = 95;
const int topBunAngle = 95;
const int botBunAngle = 93;
const int plateAngle = 77;
const int fullRetract = 27;
const int updownSpeed = 2048;
volatile int currentAngle = homeAngle;

const unsigned long upTime = 2750;
const int servoSpeed = 150;
const int stopPW = 1500;
const int CWPW = 1300;
const int CCWPW = 1700;
const int foodPlatformAngle = 0;
const int platePlatformAngle = 0;
Servo clawServo;
Servo pinionServo;

//rotary
int upToClicksG;
volatile int clickL;
volatile int clickR;
volatile int clickCounterL;
volatile int clickCounterR;
volatile int prev_clickL;
volatile int prev_clickR;
volatile int IRCounter = 0;
volatile int position_difference = 100;
volatile bool lineFollowFlag = true;

//LEDs
volatile bool LED7Flag = false;
volatile bool LED8Flag = false;

//task handling
TaskHandle_t LEDHandle;
// TaskHandle_t goToFirstHandle;
// TaskHandle_t goToCounterHandle;
// TaskHandle_t grabAndStackHandle;
// TaskHandle_t serveHandle;

//functions

void startUp();
//flashes on reset

void shutDown();
//permanetly stops loop until reset

void stop();
//stops the driver wheels

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right);
//sets speed and direction for driver wheels

void turn(char direction);
//turns the driver wheels

void lastTurn(char direction);
//turns the driver wheels

void linefollow(char motor_direction);
//follows line for the provided time

void linefollowTimer(char motorDirection, unsigned long time);

void goTo(int initialPosition, int finalPosition);
//goes from one line to another

void upTo(int upTo_delay);
//moves up to counter

void backUp();
//goes back to main black line

void locateServeArea(int currentPosition);
//goes to horizontal position of serving area

double getError();
//locates position of IRs on the black tape

void grab(String food);
//grabs food or plate

void stack(String food);
//stacks held item on plate

void cook();
//holds food on stove for 10 seconds

void stackOnPlatform(String food);
//stacks food on platform

void servoMove(int angle);
//moves the servo to a specified angle

void clawUp();
//manually moves claw up

void resetRotaryCount();
//resets rotary count

void clickCountLeft();
//counts rotary clicks on the left wheel

void clickCountRight();
//counts rotary clicks on the right wheel

void IRCount();
//counts number of activations from wing IR sensors

void toggleLED(void *params);
//Toggles LED

void changeMUX(bool S0, bool S1, bool S2);
//change multiplexer gate

void moveToNextCounter(int initialPosition, int finalPosition, char firstTurn, char secondTurn, int upToClicks);
//moves from one counter to another

void grabAndStack(String food, char platformIncluded);
//grabs and stacks food

void goToServe(int initialPostion);
//goes to serve area

void goToFirstCounter(int finalPosition, char side, int upToClicks);
//goes to counter from startinf position

void cheesePlate();
//makes cheese plate

void salad();
//makes salad


//set up

void setup() {
  
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(IR_sideLeft), IRCount, HIGH);
  attachInterrupt(digitalPinToInterrupt(IR_sideRight), IRCount, HIGH);
  attachInterrupt(digitalPinToInterrupt(rotaryPin1), clickCountLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin2), clickCountLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin3), clickCountRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryPin4), clickCountRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(clawMovePin), clawUp, HIGH);
  
  //input
  pinMode(IR_sideRight, INPUT_PULLUP);
  pinMode(IR_sideLeft, INPUT_PULLUP);
  pinMode(IR_farRight, INPUT_PULLUP);
  pinMode(IR_right, INPUT_PULLUP);
  pinMode(IR_left, INPUT_PULLUP);
  pinMode(IR_farLeft, INPUT_PULLUP);
  pinMode(microswitch, INPUT_PULLDOWN);
  pinMode(limitSwitch, INPUT_PULLDOWN);
  pinMode(clawMovePin, INPUT);
  pinMode(rotaryPin1, INPUT_PULLUP);
  pinMode(rotaryPin2, INPUT_PULLUP);
  pinMode(rotaryPin3, INPUT_PULLUP);
  pinMode(rotaryPin4, INPUT_PULLUP);

  //output
  pinMode(MUX1, OUTPUT);
  pinMode(MUX2, OUTPUT);
  pinMode(MUX3, OUTPUT);
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
  clawServo.attach(servo1IN);
  clawServo.write(homeAngle);
  pinionServo.attach(servo2IN);

  //freeRTOS
  xTaskCreate(toggleLED, "toggleLED", 2048, NULL, 1, &LEDHandle);
  // xTaskCreate(goToFirstCounter, "goToFirst", 2048, NULL, 1, &goToFirstHandle);
  // xTaskCreate(goToServe, "serve", 2048, NULL, 1, &serveHandle);
  // xTaskCreate(moveToNextCounter, "goToCounter", 2048, NULL, 1, &goToCounterHandle);
  // xTaskCreate(grabAndStack, "grabAndStack", 2048, NULL, 1, &grabAndStackHandle);
  
}


//loop

void loop() {
  startUp();
  //ledcWrite(clawCH1, updownSpeed);
  //ledcWrite(clawCH2, 0);
  cheesePlate();
  shutDown();
}

//function definitions

void startUp(){
  LED8Flag = true;
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  LED8Flag = false;
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  LED8Flag = true;
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  LED8Flag = false;
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  LED7Flag = true;
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  LED7Flag = false;
}

void shutDown(){
  stop();
  while(1){LED8Flag = true;}
}

void stop(){
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
  ledcWrite(CH3, 0);
  ledcWrite(CH4, 0);
}

void setSpeed(char left_motor, char right_motor, int speed_left, int speed_right){
  if(left_motor == 'f'){
    ledcWrite(CH1, constrain(speed_left, 0, (1 << 12) - 1));
    ledcWrite(CH2, 0);
  } else if (left_motor == 'b'){
    ledcWrite(CH1, 0);
    ledcWrite(CH2, constrain(speed_left, 0, (1 << 12) - 1));
  }
  if(right_motor == 'f'){
    ledcWrite(CH3, constrain(speed_right, 0, (1 << 12) - 1));
    ledcWrite(CH4, 0);
  } else if (right_motor == 'b'){
    ledcWrite(CH3, 0);
    ledcWrite(CH4, constrain(speed_right, 0, (1 << 12) - 1));
  }
}

void turn(char direction){
  LED7Flag = true;
  if(direction == 'r'){
    setSpeed('b','f', set_speed, set_speed);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_right) && getError() != -0.01){
      setSpeed('b','f', turn_speed, turn_speed);
    }
    stop();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_right) && getError() != -0.01){
      setSpeed('f','b', turn_speed, turn_speed);
    } 
  } else if(direction == 'l'){
    setSpeed('f','b', set_speed, set_speed);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_left) && getError() != -0.01){
      setSpeed('f','b', turn_speed, turn_speed);
    }
    stop();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_left) && getError() != -0.01){
        setSpeed('b','f', turn_speed, turn_speed);
      }
    }
    stop();
    LED7Flag = false;
}

void lastTurn(char direction){
  LED7Flag = true;
  if(direction == 'r'){
    setSpeed('b','f', set_speed, set_speed);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_sideLeft) && getError() != -0.01){
      setSpeed('b','f', turn_speed, turn_speed);
    }
    stop();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_sideLeft) && getError() != -0.01){
      setSpeed('f','b', turn_speed, turn_speed);
    } 
  } else if(direction == 'l'){
    setSpeed('f','b', set_speed, set_speed);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_sideRight) && getError() != -0.01){
      setSpeed('f','b', turn_speed, turn_speed);
    }
    stop();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    while(!digitalRead(IR_sideRight) && getError() != -0.01){
        setSpeed('b','f', turn_speed, turn_speed);
      }
    }
    stop();
    LED7Flag = false;
}


void linefollow(char motorDirection){
while(lineFollowFlag){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 0: setSpeed('b','b', set_speed * 0.85, set_speed * 0.85); break;
    case 8: setSpeed('f','f', set_speed, 0.65); break;
    case 12: setSpeed('f','f', set_speed, set_speed * 0.75); break;
    case 14: setSpeed('f','f', set_speed, set_speed * 0.8); break;
    case 4: setSpeed('f','f', set_speed, set_speed * 0.9); break;
    case 6: setSpeed('f','f', set_speed, set_speed); break;
    case 2: setSpeed('f','f', set_speed * 0.9, set_speed); break;
    case 7: setSpeed('f','f', set_speed * 0.8, set_speed); break;
    case 3: setSpeed('f','f', set_speed * 0.75, set_speed); break;
    case 1: setSpeed('f','f', 0.65, set_speed); break;
    case 15: setSpeed('b','b', set_speed, set_speed); vTaskDelay(300 / portTICK_PERIOD_MS);
    lineFollowFlag = false; setSpeed('b','b', 0, 0);break;
    default: {
      setSpeed('f','f', set_speed, set_speed);
    }
  }
 }
 lineFollowFlag = true;
}

void linefollowTimer(char motorDirection, unsigned long time){
int set_speedT = 1200;
unsigned long initialTimer = millis(), currentTime = millis();
while(time > currentTime - initialTimer){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 0: setSpeed('f','f', set_speedT, set_speedT); break;
    case 8: setSpeed('f','f', set_speedT, 0.65); break;
    case 12: setSpeed('f','f', set_speedT, set_speedT * 0.75); break;
    case 14: setSpeed('f','f', set_speedT, set_speedT * 0.8); break;
    case 4: setSpeed('f','f', set_speedT, set_speedT * 0.9); break;
    case 6: setSpeed('f','f', set_speedT, set_speedT); break;
    case 2: setSpeed('f','f', set_speedT * 0.9, set_speedT); break;
    case 7: setSpeed('f','f', set_speedT * 0.8, set_speedT); break;
    case 3: setSpeed('f','f', set_speedT * 0.75, set_speedT); break;
    case 1: setSpeed('f','f', 0.65, set_speedT); break;
    case 15: setSpeed('f','f', 0, 0); break;
    default: {
      setSpeed('f','f', set_speedT, set_speedT);
    }
  }
  currentTime = millis();
 }
}

void goTo(int initialPosition, int finalPosition){
  LED7Flag = true;
  IRCounter = 0;
  position_difference = abs(finalPosition - initialPosition);
  char motorDirection = initialPosition < finalPosition ? 'f' : 'b';

  linefollow(motorDirection);

  lineFollowFlag = true;
  setSpeed('b','b',set_speed,set_speed);
  vTaskDelay(250 / portTICK_PERIOD_MS);
  stop();
  LED7Flag = false;
}

void upTo(int upToClicks){
  upToClicksG = upToClicks;
  LED7Flag = true;
  resetRotaryCount();
  linefollow('f');
  lineFollowFlag = true;
  stop();
  LED7Flag = false;
}

void backUp(){
  LED7Flag = true;
  linefollowTimer('f',250);
  while(!digitalRead(IR_sideLeft)){
    setSpeed('b','b',set_speed,set_speed);
  }
  setSpeed('f','f',set_speed,set_speed);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  stop();
  LED7Flag = false;
}

void locateServeArea(int currentPosition){
  LED7Flag = true;
  int distance_to_area;
  if(currentPosition == 1 || currentPosition == 6){
    distance_to_area = serve_area_far;
  } else if(currentPosition == 4){
    distance_to_area = serve_area_close;
  }
  linefollowTimer('f', distance_to_area);
  stop();
  LED7Flag = false;
}

double getError(){
  int farLeftIR_reading = digitalRead(IR_farLeft);
  int leftIR_reading = digitalRead(IR_left);
  int rightIR_reading = digitalRead(IR_right);
  int farRightIR_reading = digitalRead(IR_farRight);
  int bitSum = 8 * farLeftIR_reading + 4 * leftIR_reading + 2 * rightIR_reading + 1 * farRightIR_reading;
  switch(bitSum){
    case 0: return -10; break;
    case 8: return 4; break;
    case 12: return 3; break;
    case 14: return 2; break;
    case 4: return 1; break;
    case 6: return 0; break;
    case 2: return -1; break;
    case 7: return -2; break;
    case 3: return -3; break;
    case 1: return -4; break;
    case 15: return -0.01; break;
    default: {
      LED7Flag = true;
      //vTaskDelay(250/portTICK_PERIOD_MS);
      return 0;
    }
  }
}

void grab(String food){
  LED7Flag = true;
  int finalAngle;
  if(food == "lecttuce"){
    finalAngle = lettuceAngle;
  } else if (food == "tomato"){
    finalAngle = tomatoAngle;
  } else if (food == "patty"){
    finalAngle = pattyAngle;
  } else if (food == "bun"){
    finalAngle = bunAngle;
  } else if (food == "cheese"){
    finalAngle = cheeseAngle; 
  } else if (food == "plate"){
    finalAngle = plateAngle;
  }
  for(int i = currentAngle; i >= homeAngle; i--){
    clawServo.write(i);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  currentAngle = homeAngle;
  while(!digitalRead(microswitch)){
    ledcWrite(clawCH1, 0);
    ledcWrite(clawCH2, updownSpeed);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  ledcWrite(clawCH2, 0);
  for(int i = currentAngle; i <= finalAngle; i++){
    clawServo.write(i);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  currentAngle = finalAngle;
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  vTaskDelay(upTime / portTICK_PERIOD_MS);
  ledcWrite(clawCH1, 0);
  LED7Flag = false;
}

void stack(String food){
  LED7Flag = true;
  ledcWrite(clawCH1, 0);
  ledcWrite(clawCH2, updownSpeed);
  if(food == "lecttuce"){
    vTaskDelay(700 / portTICK_PERIOD_MS);
  } else if (food == "tomato"){
    vTaskDelay(800 / portTICK_PERIOD_MS);
  } else if (food == "patty"){
    vTaskDelay(900 / portTICK_PERIOD_MS);
  } else if (food == "topbun"){
    vTaskDelay(500 / portTICK_PERIOD_MS);
  } else if (food == "bottombun"){
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } else if (food == "cheese"){
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } else if (food == "plate"){
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
  ledcWrite(clawCH2, 0);
  for(int i = currentAngle; i >= homeAngle; i--){
    clawServo.write(i);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  currentAngle = homeAngle;
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  vTaskDelay(upTime / portTICK_PERIOD_MS);
  ledcWrite(clawCH1, 0);
  LED7Flag = false;
}

void cook(){
  LED7Flag = true;
  while(!digitalRead(microswitch)){
    ledcWrite(clawCH1, 0);
    ledcWrite(clawCH2, updownSpeed);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  ledcWrite(clawCH2, 0);
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  ledcWrite(clawCH1, updownSpeed);
  ledcWrite(clawCH2, 0);
  vTaskDelay(upTime / portTICK_PERIOD_MS);
  ledcWrite(clawCH1, 0);
}

void stackOnPlatform(String food){
  pinionServo.writeMicroseconds(CWPW);
  while(!digitalRead(limitSwitch)){
  vTaskDelay(100/portTICK_PERIOD_MS);
  }
  if(food != "plate"){
    servoMove(foodPlatformAngle);
  } else if (food == "plate"){
    servoMove(platePlatformAngle);
  }
  stack(food);
  pinionServo.writeMicroseconds(CWPW);
  while(!digitalRead(limitSwitch)){
  vTaskDelay(100/portTICK_PERIOD_MS);
  }
  pinionServo.writeMicroseconds(stopPW);
}

void servoMove(int angle){
  int pulseWidth;
  if(angle > 0){
    pulseWidth = CWPW;
  } else if(angle < 0){
    pulseWidth = CCWPW;
  } else {
    pulseWidth = stopPW;
  }
  double rotationPeriod = (abs(angle) / servoSpeed) * 6000;
  pinionServo.writeMicroseconds(pulseWidth);
  vTaskDelay(rotationPeriod);
  pinionServo.writeMicroseconds(stopPW);
}

void clawUp(){
  while(digitalRead(clawMovePin)){
    if(!digitalRead(microswitch)){
      ledcWrite(clawCH1, updownSpeed);
      ledcWrite(clawCH2, 0);
    }
    if(digitalRead(microswitch)){
      ledcWrite(clawCH2, updownSpeed);
      ledcWrite(clawCH1, 0);
    }
  }
  ledcWrite(clawCH2, 0);
  ledcWrite(clawCH1, 0);
}

void resetRotaryCount(){
  clickCounterL = 0;
  prev_clickL = 0;
  clickCounterR = 0;
  prev_clickR = 0;
}

void clickCountLeft(){
  clickL = 2 * digitalRead(rotaryPin1) + 1 * digitalRead(rotaryPin2);
  if(clickL == 1 && prev_clickL == 3 || clickL == 0 && prev_clickL == 1 ||
   clickL == 2 && prev_clickL == 0 || clickL == 3 && prev_clickL == 2){
    clickCounterL--;
   } else if (clickL == 3 && prev_clickL == 1 || clickL == 2 && prev_clickL == 3 ||
   clickL == 0 && prev_clickL == 2 || clickL == 1 && prev_clickL == 0){
    clickCounterL++;
   }
  prev_clickL = clickL;
  if((abs(clickCounterL) + abs(clickCounterR)) / 2 == upToClicksG){
    lineFollowFlag == false;
  }
}

void clickCountRight(){
  clickR = 2 * digitalRead(rotaryPin3) + 1 * digitalRead(rotaryPin4);
  if(clickR == 1 && prev_clickR == 3 || clickR == 0 && prev_clickR == 1 ||
   clickR == 2 && prev_clickR == 0 || clickR == 3 && prev_clickR == 2){
    clickCounterR--;
   } else if (clickR == 3 && prev_clickR == 1 || clickR == 2 && prev_clickR == 3 ||
   clickR == 0 && prev_clickR == 2 || clickR == 1 && prev_clickR == 0){
    clickCounterR++;
   }
  prev_clickR = clickR;
  if((abs(clickCounterL) + abs(clickCounterR)) / 2 == upToClicksG){
    lineFollowFlag == false;
  }
}

void IRCount(){
  if(((digitalRead(IR_sideLeft) == HIGH || digitalRead(IR_sideRight)) == HIGH) && millis() - prevTime > 1000){
    IRCounter++;
    prevTime = millis();
    if(IRCounter == position_difference){
      lineFollowFlag = false;
    }
  }
}

void toggleLED(void *params){
  //robot2
  while(1) { 
      if (digitalRead(IR_farRight)) {
        changeMUX(0, 0, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (digitalRead(IR_right)) {
        changeMUX(0, 1, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (digitalRead(IR_left)) {
        changeMUX(0, 0, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (digitalRead(IR_farLeft)) {
        changeMUX(0, 1, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (digitalRead(IR_sideLeft)) {
        changeMUX(1, 0, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      if (digitalRead(IR_sideRight)) {
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
  /*
  //robot1
  while(1) { 
    if (digitalRead(IR_farRight)) {
      changeMUX(0, 1, 0);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    if (digitalRead(IR_right)) {
      changeMUX(1, 0, 0);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    if (digitalRead(IR_left)) {
      changeMUX(0, 0, 0);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    if (digitalRead(IR_farLeft)) {
      changeMUX(1, 1, 0);
      vTaskDelay(1/portTICK_PERIOD_MS);
      }
    if (digitalRead(IR_sideLeft)) {
      changeMUX(0, 0, 1);
      vTaskDelay(1/portTICK_PERIOD_MS);
      }
    if (digitalRead(IR_sideRight)) {
      changeMUX(1, 0, 1);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    if (LED7Flag) {
      changeMUX(0, 1, 1);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
    if (LED8Flag) {
      changeMUX(1, 1, 1);
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
  }
  */
}

void changeMUX(bool S0, bool S1, bool S2){
  digitalWrite(MUX1, S0);
  digitalWrite(MUX2, S1);
  digitalWrite(MUX3, S2);
}

void moveToNextCounter(int initialPosition, int finalPosition, char firstTurn, char secondTurn, int upToClicks){
  backUp();
  turn(firstTurn);
  goTo(initialPosition, finalPosition);
  turn(secondTurn);
  upTo(upToClicks);
}

void grabAndStack(String food, char platformIncluded){
  grab(food);
  if(platformIncluded == 'y'){
  stackOnPlatform(food);
  } else if (platformIncluded == 'n'){
    stack(food);
  }
}

void goToServe(int initialPostion){
  LED7Flag = true;
  backUp();
  if(initialPostion == 1){
    turn('l');
    locateServeArea(1);
    turn('r');
    setSpeed('f','f',2300,2300);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } else if (initialPostion == 6){
    turn('r');
    locateServeArea(6);
    turn('l');
    setSpeed('f','f',2300,2300);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } else if (initialPostion == 4){
    turn('l');
    locateServeArea(4);
    turn('l');
    setSpeed('f','f',2300,2300);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  stop();
  LED7Flag = false;
}

void goToFirstCounter(int finalPosition, char side, int upToClicks){
  goTo(0,finalPosition);
  turn(side);
  upTo(upToClicks);
}

void cheesePlate(){
  goTo(0,1);
  turn('r');
  linefollowTimer('f', 800);
  grab("cheese"); 
  backUp();
  turn('l');
  goTo(1,6);
  turn('l');
  linefollowTimer('f', 2000);
  stack("cheese");
  grab("plate");
  backUp();
  turn('l');
  locateServeArea(1);
  lastTurn('l');
  linefollowTimer('f', 2000);
  stop();
  stack("plate");
}

void salad(){
  goTo(0,2);
  turn('l');
  upTo(24);
  grab("tomato");
  backUp();
  turn('r');
  goTo(2,6);
  turn('l');
  upTo(24);
  stack("tomato");
  grab("plate");
  backUp();
  stackOnPlatform("plate");
  turn('r');
  turn('r');
  upTo(24);
  grab("lettuce");
  backUp();
  stack("lettuce");
  turn('r');
  locateServeArea(1);
  turn('l');
  upTo(24);
  grab("plate");
  stack("plate");
}