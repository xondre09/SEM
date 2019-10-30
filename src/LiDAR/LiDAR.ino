/**
 * @file    LiDAR.ino
 * @author  Filip Januš (xjanusXX)
 * @author  Marek Barvíř (xbarviXX)
 * @author  Karel Ondřej (xondre09)
 * @date    18. 10. 2019
 * 
 * @brief   2D LiDAR with ESP-WROOM-32
 */

#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LIDARLite.h>
#include <SPIFFS.h>


#define EEPROM_SIZE (2*sizeof(int))
#define EEPROM_MEASUREMENT_COUNT_OFFSET 0
#define EEPROM_PERIOD_OFFSET sizeof(int)

#define MIN_MEASUREMENT_COUNT 1
#define MAX_MEASUREMENT_COUNT 720
#define MEASUREMENT_COUNT_DEFAULT 360

#define MIN_PERIOD 500
#define MAX_PERIOD 1500
#define PERIOD_DEFAULT 1000

int measurementCount; 
int period; 
int measurementInterval;

const char ssid[] = "SEM-LiDAR";
const char password[] = "semprojekt";

hw_timer_t* timer = NULL;
bool doMeasurement;

int measurementNumber = 0;
int buffer[MAX_MEASUREMENT_COUNT];

AsyncWebServer server(80);
LIDARLite lidar;


/********************************************************************************
 * Buffer
 *******************************************************************************/
/**
 * Clear buffer.
 */
void clearBuffer()
{
  for (int i = 0; i < measurementCount; ++i)
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
 * EEPROM 
 *******************************************************************************/
/**
 * Load settings from EEPROM.
 */
void loadSettings()
{
  Serial.print("Load settings: ");
  EEPROM.get(EEPROM_MEASUREMENT_COUNT_OFFSET, measurementCount);
  EEPROM.get(EEPROM_PERIOD_OFFSET, period);

  if (measurementCount <= 0 || MAX_MEASUREMENT_COUNT < measurementCount)
  {
    measurementCount = MEASUREMENT_COUNT_DEFAULT;
  }
  if (period < MIN_PERIOD || MAX_PERIOD < period)
  {
    period = PERIOD_DEFAULT;
  }
  
  Serial.println("done");
  Serial.print("Measurement count: ");
  Serial.println(measurementCount);
  Serial.print("Period: ");
  Serial.print(period);
  Serial.println(" ms");
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
 * Update measurement interval and set timer alarm. 
 */
void updateMeasurementInterval()
{
  measurementInterval = (period*1000) / measurementCount;
  timerAlarmWrite(timer, measurementInterval, true);
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
  updateMeasurementInterval();
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
 * SERVER
 *******************************************************************************/
/**
 * Configuration of access point.
 */
void serverSetup()
{
  Serial.print("Configuring access point: ");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("done");
  
  IPAddress myIP = WiFi.softAPIP(); 

  Serial.print("HTTP server initialization: ");
  server.on("/data", handleData);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.serveStatic("/js/", SPIFFS, "/js/");
  server.serveStatic("/css/", SPIFFS, "/css/");
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
void handleData(AsyncWebServerRequest *request)
{
  String json = "{\"size\":";
  json += measurementCount;
  json += ",\"data\":[";

  int distance;
  for (int idx = 0; idx < measurementCount; ++idx) 
  {
    distance = buffer[idx];

    json += distance;
    
    if (idx + 1 != measurementCount)
    {
      json += ",";
    }
  }
  json += "]}";

  request->send(200, "application/json", json);
}

/**
 * Change settings.
 */
void handleSettings(AsyncWebServerRequest *request)
{
  Serial.println("Settings:");
  int value;
  if (request->hasArg("frames"))
  {
    value = request->arg("frames").toInt();
    if (value < MIN_MEASUREMENT_COUNT || MAX_MEASUREMENT_COUNT < value)
    {
      Serial.print("Measurement count must be in interval <");
      Serial.print(MIN_MEASUREMENT_COUNT);
      Serial.print(", ");
      Serial.print(MAX_MEASUREMENT_COUNT);
      Serial.println(">.");
    }
    else
    {
      measurementCount = value;
      EEPROM.put(EEPROM_MEASUREMENT_COUNT_OFFSET, measurementCount);
    }
  }

  if (request->hasArg("speed"))
  {
    value = request->arg("speed").toInt();
    if (value < MIN_PERIOD|| MAX_PERIOD < value)
    {
      Serial.print("Period must be in interval <");
      Serial.print(MIN_PERIOD);
      Serial.print(", ");
      Serial.print(MAX_PERIOD);
      Serial.println(">.");
    }
    else
    {
      period = value;
      EEPROM.put(EEPROM_PERIOD_OFFSET, period);
    }
  }
  EEPROM.commit();
  clearBuffer();
  updateMeasurementInterval();

  Serial.print("Measurement count: ");
  Serial.println(measurementCount);
  Serial.print("Period: ");
  Serial.print(period);
  Serial.println(" ms");
  Serial.print("Timer interval: ");
  Serial.print(measurementInterval);
  Serial.println(" us");

  
  request->send(200);
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

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  clearBuffer();
   
  Serial.print("File system initialization: ");  
  SPIFFS.begin();
  Serial.println("done");

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
  if (doMeasurement)
  {
    distanceMeasurement();
    doMeasurement = false;
    measurementNumber += 1;
    measurementNumber %= measurementCount;
  }
}
