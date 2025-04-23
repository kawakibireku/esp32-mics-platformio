#include "Adafruit_CCS811.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <Arduino.h>
#include <config.h>
#include <MICS6814.h>
#include <PubSubClient.h>
#include "esp_wpa2.h"
#include <SPIFFS.h>
#include <time.h>

Adafruit_CCS811 ccs;
Adafruit_SSD1306 display(SSD_SCREEN_WIDTH, SSD_SCREEN_HEIGHT, &Wire, SSD_OLED_RESET);
const char* mqtt_server = MQTT_SERVER;
const char* mqtt_username = MQTT_USER;
const char* mqtt_password = MQTT_PASS;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char* identity = WIFI_IDENTITY;

int mode = 1;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;  // GMT+7 (Jakarta)
const int daylightOffset_sec = 0;
const char* dataFile = "/sensor_data.csv";
unsigned long lastTimeSync = 0;
const unsigned long timeResyncInterval = 3600000; // Resync NTP time every hour
bool ntpSynced = false;

WiFiClient espClient;
PubSubClient client(espClient);

// Initialize SPIFFS storage
bool initStorage() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed");
    return false;
  }
  
  // Create file with headers if it doesn't exist
  if (!SPIFFS.exists(dataFile)) {
    File file = SPIFFS.open(dataFile, FILE_WRITE);
    if (file) {
      file.println("timestamp,device_id,CO,NO2,CO2,TVOC");
      file.close();
      Serial.println("Data file created with headers");
    }
  }
  
  return true;
}

// Get current timestamp as string
String getTimestamp() {
  struct tm timeinfo;
  char timestamp[25];
  
  if (!getLocalTime(&timeinfo)) {
    // If time not available, use millis as fallback
    sprintf(timestamp, "T%lu", millis());
  } else {
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }
  
  return String(timestamp);
}

// Save sensor data to SPIFFS
void saveDataToStorage(float CO, float NO2, float CO2, float TVOC) {
  File file = SPIFFS.open(dataFile, FILE_APPEND);
  if (file) {
    String timestamp = getTimestamp();
    String dataLine = timestamp + "," + String(DEVICE_NAME) + "," + 
                     String(CO, 2) + "," + String(NO2, 2) + "," + 
                     String(CO2, 2) + "," + String(TVOC, 2);
    
    file.println(dataLine);
    file.close();
    Serial.println("Data saved to storage: " + dataLine);
  } else {
    Serial.println("Failed to open data file for writing");
  }
}

// Attempt to send stored data when connection is restored
void sendStoredData() {
  if (!client.connected()) {
    return;
  }
  
  File file = SPIFFS.open(dataFile, FILE_READ);
  if (!file) {
    Serial.println("No stored data file found");
    return;
  }
  
  // Create a temporary file for the remaining data
  File tempFile = SPIFFS.open("/temp.csv", FILE_WRITE);
  if (!tempFile) {
    file.close();
    Serial.println("Failed to create temp file");
    return;
  }
  
  // Copy header to temp file
  String line = file.readStringUntil('\n');
  tempFile.println(line);
  
  // Process each data line
  int sentCount = 0;
  while (file.available()) {
    line = file.readStringUntil('\n');
    if (line.length() > 0) {
      // Parse the line
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);
      int fourthComma = line.indexOf(',', thirdComma + 1);
      int fifthComma = line.indexOf(',', fourthComma + 1);
      
      if (firstComma > 0 && secondComma > 0 && thirdComma > 0 && fourthComma > 0 && fifthComma > 0) {
        String timestamp = line.substring(0, firstComma);
        String deviceId = line.substring(firstComma + 1, secondComma);
        String CO = line.substring(secondComma + 1, thirdComma);
        String NO2 = line.substring(thirdComma + 1, fourthComma);
        String CO2 = line.substring(fourthComma + 1, fifthComma);
        String TVOC = line.substring(fifthComma + 1);
        
        // Create MQTT payloads
        char payload1[256], payload2[256];
        snprintf(payload1, sizeof(payload1), "emission,device_id=%s,timestamp=%s CO=%.2f,NO2=%.2f", 
                deviceId.c_str(), timestamp.c_str(), CO.toFloat(), NO2.toFloat());
        snprintf(payload2, sizeof(payload2), "emission,device_id=%s,timestamp=%s CO2=%.2f,TVOC=%.2f", 
                deviceId.c_str(), timestamp.c_str(), CO2.toFloat(), TVOC.toFloat());
        
        // Try to publish
        if (client.publish("egcs/egc-1", payload1) && client.publish("egcs/egc-1", payload2)) {
          sentCount++;
          Serial.println("Sent stored data: " + line);
          delay(100); // Avoid flooding the server
        } else {
          // If publish fails, save this line and all remaining lines to temp file
          tempFile.println(line);
          break;
        }
      }
    }
  }
  
  // Copy any remaining lines to the temp file
  while (file.available()) {
    line = file.readStringUntil('\n');
    if (line.length() > 0) {
      tempFile.println(line);
    }
  }
  
  file.close();
  tempFile.close();
  
  // Replace the original file with the temp file if we sent at least one record
  if (sentCount > 0) {
    SPIFFS.remove(dataFile);
    SPIFFS.rename("/temp.csv", dataFile);
    Serial.println("Sent " + String(sentCount) + " stored records");
  } else {
    SPIFFS.remove("/temp.csv");
  }
}

// Sync time with NTP
void syncTimeWithNTP() {
  unsigned long currentMillis = millis();
  
  if (WiFi.status() == WL_CONNECTED && 
     (!ntpSynced || (currentMillis - lastTimeSync > timeResyncInterval))) {
    
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[30];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.print("NTP time sync successful: ");
      Serial.println(timeStr);
      
      lastTimeSync = currentMillis;
      ntpSynced = true;
    } else {
      Serial.println("Failed to obtain time from NTP server");
    }
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  WiFi.disconnect(true);      
  // esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)identity, strlen(identity));
  // esp_wifi_sta_wpa2_ent_set_username((uint8_t *)identity, strlen(identity));
  // esp_wifi_sta_wpa2_ent_set_password((uint8_t *)password, strlen(password));
  // esp_wifi_sta_wpa2_ent_enable();
  // WPA2 enterprise magic ends here

  // WiFi.begin(ssid, WPA2_AUTH_PEAP, identity, identity, password); // WPA2 enterprise magic
  WiFi.begin(ssid, password); //(WPA2 personal magic)
  


  int maxAttempts = 10;
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts) {
    delay(1000);
    Serial.print(F("."));
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect. Please check credentials.");
  }
}

void reconnectMQTT() {
   while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      // client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void monitorMICS() {
  static unsigned long lastPublishTime = 0;
  static unsigned long lastSaveTime = 0;
  unsigned long currentTime = millis();
  float COval, NO2val;
  COval = ppmToUgM3(CO);
  NO2val = ppmToUgM3(NO2);
  Serial.println("----------------------");
  Serial.println("MICS6814 Sensor Readings:");
  
  Serial.print("CO: ");
  Serial.print(getResistance(CH_RED));
  Serial.print("/");
  Serial.print(getBaseResistance(CH_RED));
  Serial.print(" = ");
  Serial.print(getCurrentRatio(CH_RED));
  Serial.print(" => ");  
  Serial.print(measureMICS(CO));
  Serial.println(" ppm");
  Serial.print("CO (ug/m3): ");
  Serial.println(ppmToUgM3(CO));
  delay(50);

  Serial.print("NO2: ");
  Serial.print(getResistance(CH_OX));
  Serial.print("/");
  Serial.print(getBaseResistance(CH_OX));
  Serial.print(" = ");
  Serial.print(getCurrentRatio(CH_OX));
  Serial.print(" => ");  
  Serial.print(measureMICS(NO2));
  Serial.println(" ppm");
  Serial.print("NO2 (ug/m3): ");
  Serial.println(ppmToUgM3(NO2));
  Serial.println("----------------------");

  // Store data or publish every 10 seconds
  if(currentTime - lastSaveTime >= 10000) {
    bool published = false;
    
    // Try to publish if connected
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      char payload[256];
      snprintf(payload, sizeof(payload), "emission,device_id=%s CO=%.2f,NO2=%.2f", 
              DEVICE_NAME, COval, NO2val);
      
      Serial.println("Publishing to MQTT...");
      Serial.println(payload);
      
      if (client.publish("egcs/egc-1", payload)) {
        Serial.println("Publish successful");
        published = true;
      } else {
        Serial.println("Publish failed");
      }
    } else {
      Serial.println("Network unavailable - saving data locally");
    }
    
    // Save to local storage if publish failed or no connection
    if (!published) {
      // Get CCS811 readings for complete data record
      float CO2val = 0, TVOCval = 0;
      if (ccs.available() && !ccs.readData()) {
        CO2val = ccs.geteCO2();
        TVOCval = ccs.getTVOC();
      }
      
      saveDataToStorage(COval, NO2val, CO2val, TVOCval);
    }
    
    lastSaveTime = currentTime;
    lastPublishTime = currentTime;  // Keep this for compatibility
  }

  if(mode == 2) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(20, 0);
    display.print("Air Quality");

    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print("CO:");
    display.print(COval);
    display.setTextSize(1);
    display.print(" ug/m3");

    display.setTextSize(2);
    display.setCursor(0, 45);
    display.print("NO2:");
    display.print(NO2val);
    display.setTextSize(1);
    display.print(" ug/m3");
    display.display();
  }
}

void monitorCSS811() {
  static unsigned long lastPublishTime = 0;
  static unsigned long lastSaveTime = 0;
  unsigned long currentTime = millis();
  float CO2val = 0, TVOCval = 0;
  
  if(ccs.available()){
    if(!ccs.readData()){
      CO2val = ccs.geteCO2();
      TVOCval = ccs.getTVOC();
      Serial.println("----------------------");
      Serial.println("CCS811 Sensor Readings:");
      Serial.print("CO2: ");
      Serial.print(CO2val);
      Serial.println(" ppm");
      Serial.print("TVOC: ");
      Serial.print(TVOCval);
      Serial.println(" ppb");
      Serial.println("----------------------");

      // Store data or publish every 10 seconds
      if(currentTime - lastSaveTime >= 10000) {
        bool published = false;
        
        // Try to publish if connected
        if (WiFi.status() == WL_CONNECTED && client.connected()) {
          char payload[256];
          snprintf(payload, sizeof(payload), "emission,device_id=%s CO2=%.2f,TVOC=%.2f", 
                  DEVICE_NAME, CO2val, TVOCval);
          
          if (client.publish("egcs/egc-1", payload)) {
            Serial.println("Publish successful");
            published = true;
          } else {
            Serial.println("Publish failed");
          }
        } else {
          Serial.println("Network unavailable - saving data locally");
        }
        
        // Save to local storage if publish failed or no connection
        if (!published) {
          // Get MICS readings for complete data record
          float COval = ppmToUgM3(CO);
          float NO2val = ppmToUgM3(NO2);
          
          saveDataToStorage(COval, NO2val, CO2val, TVOCval);
        }
        
        lastSaveTime = currentTime;
        lastPublishTime = currentTime;  // Keep this for compatibility
      }

      if(mode == 1) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(20, 0);
        display.print("Air Quality");

        display.setTextSize(2);
        display.setCursor(0, 20);
        display.print("CO2: ");
        display.print(CO2val);
        display.setTextSize(1);
        display.print(" ppm");

        display.setTextSize(2);
        display.setCursor(0, 45);
        display.print("TVOC: ");
        display.print(TVOCval);
        display.display();
      }
    }
    else {
      Serial.println("ERROR!");
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 5);
      display.print("ERROR!");
      display.display();
      while (1);
    }
  }
}

void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(500);

  // Initialize SPIFFS
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Initializing storage...");
  display.display();
  if (initStorage()) {
    display.setCursor(0, 10);
    display.print("Storage ready!");
    display.display();
  } else {
    display.setCursor(0, 10);
    display.print("Storage failed!");
    display.display();
  }
  delay(500);

  // Step 1: Connect to WiFi
  display.setCursor(0, 20);
  display.print("Connecting to WiFi...");
  display.display();
  connectToWiFi();
  display.setCursor(0, 30);
  display.print("WiFi connected!");
  display.display();
  delay(500);

  // After WiFi connection, sync time with NTP
  if (WiFi.status() == WL_CONNECTED) {
    display.setCursor(0, 40);
    display.print("Syncing time...");
    display.display();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      ntpSynced = true;
      display.setCursor(0, 50);
      display.print("Time synced!");
      display.display();
    }
    delay(500);
  }

  // Step 2: Initialize CCS811 Sensor
  display.setCursor(0, 60);
  display.print("Initializing CCS811...");
  display.display();
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    display.setCursor(0, 70);
    display.print("CCS811 failed!");
    display.display();
    while (1);
  }
  while (!ccs.available());
  display.setCursor(0, 70);
  display.print("CCS811 initialized!");
  display.display();
  delay(500);

  // Step 3: Initialize MICS6814 Sensor
  display.setCursor(0, 80);
  display.print("Initializing MICS6814...");
  display.display();
  initMICS(NH3PIN, COPIN, OXPIN, MICS_CALIBRATION_SECONDS, MICS_CALIBRATION_DELTA);
  calibrateMICS();
  display.setCursor(0, 90);
  display.print("MICS6814 initialized!");
  display.display();
  delay(500);

  // Step 4: Set up MQTT client
  display.setCursor(0, 100);
  display.print("Setting up MQTT...");
  display.display();
  client.setServer(mqtt_server, MQTT_PORT);
  display.setCursor(0, 110);
  display.print("MQTT setup done!");
  display.display();
  delay(500);

  // Step 5: Set up button pin
  pinMode(BUTTON_PIN, INPUT);
  display.setCursor(0, 120);
  display.print("Button setup done!");
  display.display();
  delay(500);

  // Clear display after setup
  display.clearDisplay();
  display.setCursor(25, 15);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("CCS811 Sensor");
  display.setCursor(25, 35);
  display.setTextSize(1);
  display.print("Initializing");
  display.display();
}

void loop() {
  static unsigned long buttonPressStartTime = 0;
  static bool buttonPressed = false;
  static unsigned long lastReconnectAttempt = 0;
  static bool wasConnected = false;
  static bool wasMqttConnected = false;
  unsigned long currentMillis = millis();

  // Handle button presses
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    // ... existing button code ...
  } else {
    buttonPressed = false;
  }

  // Handle WiFi reconnection
  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) {
      Serial.println("WiFi connection lost");
      wasConnected = false;
    }
    
    if (currentMillis - lastReconnectAttempt > 30000) { // Try reconnect every 30 seconds
      Serial.println("Attempting WiFi reconnection...");
      connectToWiFi();
      lastReconnectAttempt = currentMillis;
    }
  } else {
    if (!wasConnected) {
      Serial.println("WiFi reconnected");
      wasConnected = true;
      
      // Sync time after reconnection
      syncTimeWithNTP();
    }
    
    // Handle MQTT reconnection
    if (!client.connected()) {
      if (wasMqttConnected) {
        Serial.println("MQTT connection lost");
        wasMqttConnected = false;
      }
      
      if (currentMillis - lastReconnectAttempt > 5000) { // Try MQTT reconnect every 5 seconds
        Serial.println("Attempting MQTT reconnection...");
        reconnectMQTT();
        lastReconnectAttempt = currentMillis;
      }
    } else {
      if (!wasMqttConnected) {
        Serial.println("MQTT reconnected - sending stored data");
        wasMqttConnected = true;
        
        // Try to send stored data after reconnecting
        sendStoredData();
      }
      
      // Process MQTT messages
      client.loop();
    }
  }
  
  // Periodically sync time if connected
  syncTimeWithNTP();

  // Always monitor sensors regardless of connection state
  monitorMICS();
  monitorCSS811();
  delay(1000);
}


