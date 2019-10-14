/**
 * @file    LiDAR.ino
 * @author  Karel Ondřej (xondre09)
 * @date    14. 10. 2019
 * 
 * @brief   2D LiDAR
 */

#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <LIDARLite.h>
#include "FS.h"


/**
 * Record of distance measurement.
 */
struct measurementRecord {
  int angle;
  int distance;
};


int MEASUREMENT_COUNT = 360;
int MEMORY_SIZE = MEASUREMENT_COUNT * sizeof(measurementRecord);


const char ssid[] = "SEM-LiDAR";
const char password[] = "semprojekt";

ESP8266WebServer server(80);

LIDARLite lidar;


/**
 * Clear EEPROM.
 */
void clearEEPROM()
{
  for (int i = 0; i < MEASUREMENT_COUNT; ++i) {
    measurementRecord record{0, NAN};
    EEPROM.put(i, record);
  }
  EEPROM.commit();
}


/**
 * Configuration of access point.
 */
void serverSetup()
{
  
  Serial.print("Configuring access point...");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress myIP = WiFi.softAPIP();
  
  Serial.print("IP address: ");
  Serial.println(myIP);

  Serial.print("File system begin...");
  SPIFFS.begin();
  
  Serial.print("Routing...");
  server.begin();
  server.on("/data", handleData);
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/js", SPIFFS, "/js");
  Serial.print("...configuring done.");
}


/**
 * ESP setup.
 */
void setup() 
{
  delay(1000);
  Serial.begin(115200);
  
  // EEPROM setup
  EEPROM.begin(MEMORY_SIZE);
  clearEEPROM();
  
  // LiDAR-Lite setup
  lidar.begin(0, true);
  lidar.configure(0);
  
  //server setup
  serverSetup();
}


/**
 * Sending distance measurement data.
 */
void handleData()
{
}


/**
 * Main loop.
 */
void loop() 
{
}
