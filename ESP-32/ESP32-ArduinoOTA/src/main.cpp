/*
ESP32 OTA using PlatformIO
Refer to:
https://lonelybinary.com/blogs/learn/esp32-ota-using-platformio
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// gateway: 192.168.0.1
// duct ip 192.168.0.104
// scotch ip 192.168.0.105

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 105); // need this (static IP address of ESP32)
// Set your Gateway IP address
IPAddress gateway(192, 168,0, 1); // need this (can change with the router)

IPAddress subnet(255, 255, 0, 0); // need this

const char* ssid = "TP-Link_C8D1"; // ssid of the wifi they're using
const char* password = "93456593"; // wifi password

void setup(void) {
  Serial.begin(115200); // starts the serial monitor

  WiFi.mode(WIFI_STA); // need this to start the wifi

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
  //ArduinoOTA.setPassword("");
  ArduinoOTA.begin();
  Serial.println("OTA success :)");
}

void loop()
{
  // OTA Handle
  ArduinoOTA.handle();
}
