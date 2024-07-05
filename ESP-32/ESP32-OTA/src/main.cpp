#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

//yoinked from internet. pls don't steal Cass's wifi

// gateway: 192.168.0.1
// duct ip 192.168.0.104
// scotch ip 192.168.0.105

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 105);
// Set your Gateway IP address
IPAddress gateway(192, 168,0, 1);

IPAddress subnet(255, 255, 0, 0);

const char* ssid = "TP-Link_C8D1";
const char* password = "93456593";

AsyncWebServer server(80);

void setup(void) {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (!WiFi.config(local_IP, gateway, subnet)) {
  Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! You have reached ESP-32 Duct");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  //Serial.println("Static IP success");
}

void loop(void) {

}