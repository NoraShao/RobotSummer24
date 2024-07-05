/*
* Demonstration of basic communication between two ESP32s
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <esp_now.h>
#include <WiFi.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAC address of receiving ESP32
uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9d, 0x66, 0x50}; // scotch 64:b7:08:9d:66:50
//uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9c, 0x61, 0x44}; // duct 64:b7:08:9c:61:44

// peer info for ESP NOW
// (peer means two-way communication--no master and slave)
esp_now_peer_info_t peerInfo;

// can send data using a structure
// the code uploaded to each ESP32 must contain the same structure definition
typedef struct message {
  String text_message;
  unsigned long timestamp;
  uint8_t num;
} message;

message incoming;
message outgoing;

// outgoing infomation
String message_out;
uint8_t num_out;
unsigned long time_out;

// incoming information
String message_in;
uint8_t num_in;
unsigned long time_in;

// stores if data was sent successfully
String success;

// function prototypes :(
void dataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void dataReceived(const uint8_t * mac, const uint8_t *incomingData, int len);
void updateDisplay();

void setup() {
  Serial.begin(115200);

  // starts display and prints version number
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("ESP32 communcation version 1.0");
  display_handler.display();
  Serial.println("ESP32 communcation version 1.0");
  delay(1000);

  // sets device as a wifi station
  WiFi.mode(WIFI_STA);

  // initialize esp-now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;

  // initializing random number generator
    randomSeed(13);
  }

  // Register peer
  // make sure to use the correct MAC address
  memcpy(peerInfo.peer_addr, MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Error adding peer");
    return;
  }

  // register for send callback to get the status of the sent packet
  esp_now_register_send_cb(dataSent);

  // register for a callback function when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(dataReceived));

}

void loop() {

  outgoing.text_message = "Duct says hi :)";
  outgoing.num = random(1, 20);
  outgoing.timestamp = millis();

  Serial.println("sending the following message: ");
  Serial.println(outgoing.text_message);
  Serial.println(outgoing.num);
  Serial.println(outgoing.timestamp);

  esp_err_t result = esp_now_send(MAC, (uint8_t *) &outgoing, sizeof(outgoing));

  if (result == ESP_OK) {
    // Serial.print("Message ");
    // Serial.print(outgoing.num);
    Serial.println("message sent :)");
  }
  else {
    Serial.println("error sending message ");
    // Serial.println(outgoing.num);
  }
  updateDisplay();
  delay(10000);
  
  };

// when data is sent
void dataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "delivered :)";
  }
  else{
    success = "not delivered :(";
  }
}

// when data is received
void dataReceived(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Bytes received: ");
  Serial.println(len);
  message_in = incoming.text_message;
  time_in = incoming.timestamp;
  num_in = incoming.num;
}

void updateDisplay() {
  // OLED display
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);

  display_handler.setCursor(0,0);
  display_handler.println("INCOMING DATA: ");

  display_handler.setCursor(0, 15);
  display_handler.print("Message: ");
  display_handler.print(message_in);

  display_handler.setCursor(0, 35);
  display_handler.print("Random number: ");
  display_handler.print(num_in);

  display_handler.setCursor(0, 25);
  display_handler.print("Time: ");
  display_handler.print(time_in);

  display_handler.display();


  // serial
  Serial.println("INCOMING DATA");
  Serial.print("Message: ");
  Serial.println(message_in);
  Serial.print("Random number: ");
  Serial.println(num_in);
  Serial.print("Time: ");
  Serial.println(time_in);
}