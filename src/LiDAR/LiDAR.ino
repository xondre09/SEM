/**
 * @file    LiDAR.ino
 * @author  Karel Ondřej (xondre09)
 * @date    14. 10. 2019
 * 
 * @brief   2D LiDAR with ESP-WROOM-32
 */

#include <ESPmDNS.h>
#include <ESP32WebServer.h>
#include <LIDARLite.h>

#include "SPIFFS.h"

const int MEASUREMENT_COUNT = 360; // degree
const int CYCLE_TIME = 1000; // [ms]
const int MEASUREMENT_INTERVAL = CYCLE_TIME / MEASUREMENT_COUNT;

const char ssid[] = "SEM-LiDAR";
const char password[] = "semprojekt";

hw_timer_t* timer = NULL;
bool doMeasurement;

int measurementNumber = 0;
int buffer[MEASUREMENT_COUNT];

ESP32WebServer server(80);
LIDARLite lidar;


/********************************************************************************
 * Buffer
 *******************************************************************************/
/**
 * Clear buffer.
 */
void clearBuffer()
{
  for (int i = 0; i < MEASUREMENT_COUNT; ++i)
  {
    int distance = 0;
    writeMeasurementRecord(i, distance);
  }
}

/**
 * Write distance measurement record to buffer;
 */
void writeMeasurementRecord(int idx, int record)
{
  buffer[idx] = record;
}


/********************************************************************************
 * SERVER
 *******************************************************************************/
/**
 * Configuration of access point.
 */
void serverSetup()
{
  Serial.println("Configuring access point:");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("done");
  
  IPAddress myIP = WiFi.softAPIP();  
  Serial.print("File system initialization: ");  
  SPIFFS.begin();
  Serial.println("done");

  Serial.print("HTTP server initialization: ");
  server.on("/data", handleData);
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/js", SPIFFS, "/js");
  server.begin();
  Serial.println("done");

  if (! MDNS.begin("lidar"))
  {
    Serial.println("Error setting up MDNS responder!");
  }

  Serial.println("HTTP server started.");
  Serial.print("IP address: ");
  Serial.println(myIP);
}

/**
 * Sending distance measurement data in JSON format.
 */
void handleData()
{
  String json = "{\"size\":";
  json += MEASUREMENT_COUNT;
  json += ",\"data\":[";

  int distance;
  for (int idx = 0; idx < MEASUREMENT_COUNT; ++idx) 
  {
    distance = buffer[idx];

    json += distance;
    
    if (idx + 1 != MEASUREMENT_COUNT)
    {
      json += ",";
    }
  }
  json += "]}";

  server.send(200, "text/json", json);
}


/********************************************************************************
 * LiDAR
 *******************************************************************************/
/**
 * A function called after a timer interrupt.
 */
void IRAM_ATTR measurementTimerCallback()
{
  doMeasurement = true;
}
 
/**
 * Initialization of LiDAR-Lite.
 */
void lidarSetup(void)
{
  lidar.begin(0, true);
  lidar.configure(0);

  doMeasurement = true;

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &measurementTimerCallback, true);
  timerAlarmWrite(timer, MEASUREMENT_INTERVAL*1000, true);
  timerAlarmEnable(timer);
}

/**
 * Distance measurement with LiDAR-Lite.
 */
void distanceMeasurement()
{
  int distance = lidar.distance();

  writeMeasurementRecord(measurementNumber, distance);
}


/********************************************************************************
 * SETUP AND MAIN LOOP
 *******************************************************************************/
/**
 * Setup of UART, buffer, LiDAR-Lite and server.
 */
void setup() 
{
  delay(2000);
  Serial.begin(115200);

  clearBuffer();

  // server setup
  serverSetup();
  
  // LiDAR-Lite setup
  lidarSetup();
}

/**
 * Main loop.
 */
void loop() 
{
  server.handleClient();

  if (doMeasurement)
  {
    distanceMeasurement();
    doMeasurement = false;
    if (measurementNumber == 0)
    {
      Serial.print("Distance: ");
      Serial.print(buffer[0]);
      Serial.println(" cm");
    }
    measurementNumber += 1;
    measurementNumber %= MEASUREMENT_COUNT;
  }
}
