/**
 * @file    LiDAR.ino
 * @author  Karel Ondřej (xondre09)
 * @date    14. 10. 2019
 * 
 * @brief   2D LiDAR with ESP-WROOM-32
 */

#include <ESP32WebServer.h>
#include <LIDARLite.h>

#include "SPIFFS.h"


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

hw_timer_t* timer = NULL;
bool doMeasurement;

int measurementNumber = 0;
measurementRecord buffer[MEASUREMENT_COUNT];

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

  Serial.println("File system begin...");
  SPIFFS.begin();
  
  Serial.println("Routing...");
  server.begin();
  server.on("/data", handleData);
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/js", SPIFFS, "/js");
  Serial.println("...configuring done.");
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
      json += String(record.angle, 4);
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
  measurementRecord record;

  record.angle = 360.f / (float)MEASUREMENT_COUNT * (float)measurementNumber;
  record.distance = (float)lidar.distance();

  writeMeasurementRecord(measurementNumber, record);
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
      Serial.print(buffer[0].distance);
      Serial.println(" cm");
    }
    measurementNumber += 1;
    measurementNumber %= MEASUREMENT_COUNT;
  }
}
