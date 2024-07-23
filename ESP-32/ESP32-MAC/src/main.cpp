// yoinked from the internet. prints the mac address in the serial terminal

#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup(){
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin();

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
}
 
void loop(){

}


// code below is from Arduino but it prints it backwards :(((((((

// // yoinked from arduino docs https://docs.arduino.cc/retired/getting-started-guides/ArduinoWiFiShield/
// // prints the MAC address

// #include <SPI.h>
// #include <WiFi.h>

// void printMacAddress();

// void listNetworks();

// void setup() {

//   // initialize serial and wait for the port to open:

//   Serial.begin(9600);

//   //while(!Serial) ;

//   // attempt to connect using WEP encryption:
//   Serial.println("Starting Serial and getting MAC address...");
//   // Serial.println("Initializing Wifi...");

//   printMacAddress();

//   // scan for existing networks:

//   // Serial.println("Scanning available networks...");

//   // listNetworks();
// }

// void loop() {

//   // delay(10000);

//   // // scan for existing networks:

//   // Serial.println("Scanning available networks...");

//   // listNetworks();
// }

// void printMacAddress() {

//   // the MAC address of your Wifi shield

//   byte mac[6];

//   // print your MAC address:

//   WiFi.macAddress(mac);

//   Serial.print("MAC: ");

//   Serial.print(mac[5],HEX);

//   Serial.print(":");

//   Serial.print(mac[4],HEX);

//   Serial.print(":");

//   Serial.print(mac[3],HEX);

//   Serial.print(":");

//   Serial.print(mac[2],HEX);

//   Serial.print(":");

//   Serial.print(mac[1],HEX);

//   Serial.print(":");

//   Serial.println(mac[0],HEX);
// }

// void listNetworks() {

//   // scan for nearby networks:

//   Serial.println("** Scan Networks **");

//   byte numSsid = WiFi.scanNetworks();

//   // print the list of networks seen:

//   Serial.print("number of available networks:");

//   Serial.println(numSsid);

//   // print the network number and name for each network found:

//   for (int thisNet = 0; thisNet<numSsid; thisNet++) {

//     Serial.print(thisNet);

//     Serial.print(") ");

//     Serial.print(WiFi.SSID(thisNet));

//     Serial.print("\tSignal: ");

//     Serial.print(WiFi.RSSI(thisNet));

//     Serial.print(" dBm");

//     Serial.print("\tEncryption: ");

//     Serial.println(WiFi.encryptionType(thisNet));

//   }
// }