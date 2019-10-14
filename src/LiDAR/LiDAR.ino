/**
 * @file    LiDAR.ino
 * @author  Karel Ondřej (xondre09)
 * @date    14. 10. 2019
 * 
 * @brief   2D LiDAR
 */

#include <ESP8266WebServer.h>
#include <LIDARLite.h>

#include "FS.h"


/**
 * Record of distance measurement.
 */
struct measurementRecord {
  float angle;
  float distance;
};

const int MEASUREMENT_COUNT = 360; // degree
const int CYCLE_TIME = 1000; // [ms]
const int MEASUREMENT_INTERVAL = CYCLE_TIME / MEASUREMENT_COUNT;
const int MEMORY_SIZE = MEASUREMENT_COUNT * sizeof(measurementRecord);

const char ssid[] = "SEM-LiDAR";
const char password[] = "semprojekt";

os_timer_t myTimer;
bool doMeasurement;

int measurementNumber = 0;
measurementRecord buffer[MEASUREMENT_COUNT];

ESP8266WebServer server(80);
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
    measurementRecord record{0, NAN};
    writeMeasurementRecord(i, record);
  }
}

/**
 * Write distance measurement record to buffer;
 */
void writeMeasurementRecord(int idx, measurementRecord record)
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
 * Sending distance measurement data in JSON format.
 */
void handleData()
{
  String json = "{\"data\": [";

  int memoryPointer;
  measurementRecord record;
  for (int idx = 0; idx < MEASUREMENT_COUNT; ++idx) 
  {
    memoryPointer = idx * sizeof(measurementRecord);
    record = buffer[idx];

    if (! (isnan(record.angle) || isnan(record.distance)))
    {
      json += "{";
      json += "\"angle\":";
      json += record.angle;
      json += ",\"distance\":";
      json += record.distance;
      json += "}";
      
      if (idx + 1 != MEASUREMENT_COUNT)
      {
        json += ",";
      }
    }
  }
  json += "]}";

  server.send(200, "text/json", json);
}


/********************************************************************************
 * LiDAR
 *******************************************************************************/
/**
 * 
 */
void lidarSetup(void)
{
  lidar.begin(0, true);
  lidar.configure(0);

  doMeasurement = true;
  
  os_timer_setfn(&myTimer, measurementTimerCallback, NULL);
  os_timer_arm(&myTimer, MEASUREMENT_INTERVAL, true);
}

/**
 * Distance measurement with LiDAR-Lite.
 */
void distanceMeasurement()
{
  measurementRecord record;

  record.angle = (float)measurementNumber / (float)MEASUREMENT_COUNT;
  record.distance = (float)lidar.distance();

  writeMeasurementRecord(measurementNumber, record);
}

/**
 * A function called after a timer interrupt.
 */
void measurementTimerCallback(void *pArg)
{
  doMeasurement = true;
}
 

/********************************************************************************
 * SETUP AND MAIN LOOP
 *******************************************************************************/
/**
 * Setup of UART, buffer, LiDAR-Lite and server.
 */
void setup() 
{
  delay(1000);
  Serial.begin(115200);

  clearBuffer();
  
  // LiDAR-Lite setup
  lidarSetup();
  
  // server setup
  serverSetup();
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
      Serial.println(buffer[0].distance);
    }
    measurementNumber += 1;
    measurementNumber %= MEASUREMENT_COUNT;
  }
}
