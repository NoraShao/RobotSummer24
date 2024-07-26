#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define button 4
#define LED 5

// Define the MAC address of the peer (receiver in this case)
uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9d, 0x66, 0x50}; //robot 1 receiving
// uint8_t MAC[] = {0x64, 0xb7, 0x08, 0x9c, 0x61, 0x44}; //robot 2 receiving

esp_now_peer_info_t peerInfo;

typedef struct flag {
  bool act;
} flag;

flag trigAction;
bool go = false;

void sendFlag();
// Sends flag to other robot

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status);
// Confirms sendFlag

void receiveFlag();
// Holds action until flag is received

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
// Confirms flag received

void toggleState();

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error adding peer");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  pinMode(LED, OUTPUT);
  pinMode(button, INPUT);
  attachInterrupt(digitalPinToInterrupt(button), toggleState, CHANGE);
}

void loop() {
}

void sendFlag() {
  esp_err_t result = esp_now_send(MAC, (uint8_t *) &trigAction, sizeof(trigAction));
  
  if (result == ESP_OK) {
    Serial.println("Flag sent successfully");
  } else {
    Serial.println("Error sending flag");
  }
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void receiveFlag() {
  go = false;
  while (!go) {
    // Wait until flag is received
  }
}

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&trigAction, incomingData, sizeof(trigAction));
  Serial.println("Flag received");
  go = true;
}

void toggleState(){
  if(digitalRead(button) == HIGH){
    trigAction.act = true;
  }
  if(digitalRead(button) == LOW){
    trigAction.act = false;
  }
  sendFlag();
}