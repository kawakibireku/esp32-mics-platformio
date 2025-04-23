
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
#define MAX_OFFLINE_RECORDS 100
#define OFFLINE_DATA_FILE "/offline_data.txt"
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 25200   // GMT+7 (adjust based on your timezone)
#define DAYLIGHT_OFFSET_SEC 0

bool timeInitialized = false;
Adafruit_CCS811 ccs;
Adafruit_SSD1306 display(SSD_SCREEN_WIDTH, SSD_SCREEN_HEIGHT, &Wire, SSD_OLED_RESET);
const char* mqtt_server = MQTT_SERVER;
const char* mqtt_username = MQTT_USER;
const char* mqtt_password = MQTT_PASS;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char* identity = WIFI_IDENTITY;

int mode = 1;

WiFiClient espClient;
PubSubClient client(espClient);

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed!");
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("SPIFFS init failed!");
    display.display();
    delay(2000);
  }
}

// Initialize time from NTP server
void initTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  Serial.println("Waiting for NTP time sync...");
  
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 8 * 3600 * 2 && attempts < 10) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }
  
  if (now > 8 * 3600 * 2) {
    Serial.println("\nTime initialized!");
    timeInitialized = true;
    
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));
  } else {
    Serial.println("\nFailed to get time from NTP server");
  }
}

// Save data to SPIFFS when connection is unavailable
void saveOfflineData(const char* payload) {
  File file = SPIFFS.open(OFFLINE_DATA_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  
  // Add timestamp to the payload if time is initialized
  String timestampedPayload = String(payload);
  
  if (timeInitialized) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t timeMillis = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
    timestampedPayload += " " + String(timeMillis);
  } else {
    // Use millis() as fallback if NTP time is not available
    timestampedPayload += " " + String(millis());
  }
  
  file.println(timestampedPayload);
  file.close();
  
  Serial.print("Saved offline data: ");
  Serial.println(timestampedPayload);
}

// Count number of offline records
int countOfflineRecords() {
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    return 0;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, FILE_READ);
  if (!file) {
    return 0;
  }
  
  int lineCount = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.length() > 0) {
      lineCount++;
    }
  }
  
  file.close();
  return lineCount;
}

// Sync offline data when connection is restored
void syncOfflineData() {
  if (!client.connected() || !WiFi.isConnected()) {
    return;
  }
  
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    return;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  
  File tempFile = SPIFFS.open("/temp.txt", FILE_WRITE);
  if (!tempFile) {
    Serial.println("Failed to create temp file");
    file.close();
    return;
  }
  
  int syncedCount = 0;
  int failedCount = 0;
  
  Serial.println("Syncing offline data...");
  
  // Process each line in the file
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    
    if (line.length() > 0) {
      // Extract the payload (before the timestamp)
      int spacePos = line.lastIndexOf(' ');
      String payload = line;
      
      if (spacePos > 0) {
        payload = line.substring(0, spacePos);
      }
      
      // Try to publish to MQTT
      if (client.publish("egcs/egc-1", payload.c_str())) {
        syncedCount++;
        Serial.print("Synced: ");
        Serial.println(line);
        delay(100); // Small delay to avoid flooding the broker
      } else {
        failedCount++;
        tempFile.println(line);  // Keep the record for next attempt
        Serial.print("Failed to sync: ");
        Serial.println(line);
      }
    }
  }
  
  file.close();
  tempFile.close();
  
  // Replace the original file with the temp file if there are still failed records
  if (failedCount > 0) {
    SPIFFS.remove(OFFLINE_DATA_FILE);
    SPIFFS.rename("/temp.txt", OFFLINE_DATA_FILE);
  } else {
    SPIFFS.remove(OFFLINE_DATA_FILE); // All records synced, remove the file
    SPIFFS.remove("/temp.txt");
  }
  
  Serial.printf("Sync complete. Synced: %d, Failed: %d\n", syncedCount, failedCount);
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
    WiFi.begin(ssid, password);

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

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, trying to reconnect...");
    
    // Display reconnection attempt
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Reconnecting WiFi...");
    display.display();
    
    WiFi.disconnect(true);
    // WiFi.begin(ssid, WPA2_AUTH_PEAP, identity, identity, password);
    WiFi.begin(ssid, password);

    int maxAttempts = 10;
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts) {
      delay(1000);
      Serial.print(".");
      attempt++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected to WiFi!");
      display.setCursor(0, 10);
      display.print("WiFi reconnected!");
      display.display();
      delay(500);
      
      // If time wasn't initialized before, try again
      if (!timeInitialized) {
        initTime();
      }
    } else {
      Serial.println("\nFailed to reconnect WiFi");
      display.setCursor(0, 10);
      display.print("WiFi reconnect failed!");
      display.display();
      delay(500);
    }
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
  Serial.println(currentTime);
  Serial.println(lastPublishTime);
  if(currentTime - lastPublishTime >= 10000) {
    char payload[256];
    snprintf(payload, sizeof(payload), "emission,device_id=%s CO=%.2f,NO2=%.2f", DEVICE_NAME, COval, NO2val);
    Serial.println("Publishing to MQTT...");
    Serial.println(payload);
    
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      if (client.publish("egcs/egc-1", payload)) {
        Serial.println("Publish successful");
        lastPublishTime = currentTime;
      } else {
        Serial.println("Publish failed");
        saveOfflineData(payload);
      }
    } else {
      Serial.println("WiFi or MQTT not connected, saving offline");
      saveOfflineData(payload);
    }
    
    // Try to sync offline data when we have a connection
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      syncOfflineData();
    }
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
  unsigned long currentTime = millis();
  float CO2val, TVOCval;
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

        if(currentTime - lastPublishTime >= 10000) {
          // Publish to MQTT
          char payload[256];
          snprintf(payload, sizeof(payload), "emission,device_id=%s CO2=%.2f,TVOC=%.2f", DEVICE_NAME, CO2val, TVOCval);
          
          if (WiFi.status() == WL_CONNECTED && client.connected()) {
            if (client.publish("egcs/egc-1", payload)) {
              Serial.println("Publish successful");
              lastPublishTime = currentTime;
            } else {
              Serial.println("Publish failed");
              saveOfflineData(payload);
            }
          } else {
            Serial.println("WiFi or MQTT not connected, saving offline");
            saveOfflineData(payload);
          }
          
          // Try to sync offline data when we have a connection
          if (WiFi.status() == WL_CONNECTED && client.connected()) {
            syncOfflineData();
          }
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
    
        } else {
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
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3C (128x64)
  delay(500);

  // analogReadResolution(10);

  // Clear display and set initial text properties
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Initialize SPIFFS
  display.setCursor(0, 0);
  display.print("Initializing SPIFFS...");
  display.display();
  initSPIFFS();
  display.setCursor(0, 10);
  display.print("SPIFFS initialized!");
  display.display();
  delay(500);

  // Step 1: Connect to WiFi
  display.setCursor(0, 0);
  display.print("Connecting to WiFi...");
  display.display();
  connectToWiFi();
  display.setCursor(0, 10);
  display.print("WiFi connected!");
  display.display();
  delay(500);

   // Initialize time if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    display.setCursor(0, 20);
    display.print("Setting up time...");
    display.display();
    initTime();
    display.setCursor(0, 30);
    display.print("Time setup done!");
    display.display();
    delay(500);
  }

  // Step 2: Initialize CCS811 Sensor
  display.setCursor(0, 20);
  display.print("Initializing CCS811...");
  display.display();
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    display.setCursor(0, 30);
    display.print("CCS811 failed!");
    display.display();
    while (1);
  }
  while (!ccs.available());
  display.setCursor(0, 30);
  display.print("CCS811 initialized!");
  display.display();
  delay(500);

  // Step 3: Initialize MICS6814 Sensor
  display.setCursor(0, 40);
  display.print("Initializing MICS6814...");
  display.display();
  initMICS(NH3PIN, COPIN, OXPIN, MICS_CALIBRATION_SECONDS, MICS_CALIBRATION_DELTA);
  calibrateMICS();
  display.setCursor(0, 50);
  display.print("MICS6814 initialized!");
  display.display();
  delay(500);

  // Step 4: Set up MQTT client
  display.setCursor(0, 60);
  display.print("Setting up MQTT...");
  display.display();
  client.setServer(mqtt_server, MQTT_PORT);
  display.setCursor(0, 70);
  display.print("MQTT setup done!");
  display.display();
  delay(500);

  // Step 5: Set up button pin
  pinMode(BUTTON_PIN, INPUT);
  display.setCursor(0, 80);
  display.print("Button setup done!");
  display.display();
  delay(500);

  // Show offline records if any
  int offlineCount = countOfflineRecords();
  if (offlineCount > 0) {
    display.setCursor(0, 90);
    display.print("Offline records: ");
    display.print(offlineCount);
    display.display();
    delay(1000);
  }

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
  static unsigned long lastPublishTime = 0;
  static unsigned long lastSyncAttempt = 0;

  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStartTime = millis();
    } else {
      if (millis() - buttonPressStartTime >= 1000) {
        mode = 2;
        buttonPressed = false; // Reset the button pressed state
      }
    }
  } else {
    buttonPressed = false;
  }
  // Check and reconnect MQTT if needed
  if(!client.connected() && WiFi.status() == WL_CONNECTED) {
    reconnectMQTT();
  }

    // Sync offline data periodically if connected
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    if (millis() - lastSyncAttempt > 60000) { // Try every minute
      lastSyncAttempt = millis();
      syncOfflineData();
    }
  }

  // Check WiFi connection every ~30 seconds
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 30000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      reconnectWiFi();
    }
  }
  
  client.loop();

  monitorMICS();
  monitorCSS811();
  delay(1000);
}


