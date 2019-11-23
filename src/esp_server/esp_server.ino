// Code came from here: https://techtutorialsx.com/2018/06/02/esp8266-arduino-socket-server/
// Circut setup came from here: https://www.hackster.io/harshmangukiya/how-to-program-esp8266-with-arduino-uno-efb05f
// And here: https://dzone.com/articles/programming-the-esp8266-with-the-arduino-ide-in-3
// Take GPIO0 to ground to enable programming 
// Put rx and tx pins on Serial1 and ensure they match the ESP
// Make sure to take Arduino RESET pin to GND

#include "ESP8266WiFi.h"
 
const char* ssid = "DominoNet";
const char* password =  "DominoBot";
const int port = 1234;
 
WiFiServer wifiServer(port);
 
void setup() {
 
  Serial.begin(115200);
 
  delay(1000);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("<[ESP] Connecting..>");
  }
 
  Serial.print("<[ESP] Connected to WiFi. IP:");
  Serial.print(WiFi.localIP());
  Serial.print(">");
 
  wifiServer.begin();
}
 
void loop() {

  // Set up new client
  WiFiClient client = wifiServer.available();

  // The WiFiClient has overloaded bool comparison so this can evaluate true if the client is connected
  if (client) {

    // Notify host that client has connected
    Serial.print("<[ESP] Client connected>");

    // Loop forever while client is connected
    while (client.connected()) {

      // Forward any data from wifi client to serial host
      while (client.available()>0) {
        char c = client.read();
        Serial.write(c);
      }

      // Forward any data from serial host to wifi client
      while (Serial.available()>0) {
        char c = Serial.read();
        client.write(c);
      }
 
      delay(10);
    }
 
    client.stop();
    Serial.print("<[ESP] Client disconnected>");
 
  }

  Serial.print("<*>");
  delay(500);
  
}