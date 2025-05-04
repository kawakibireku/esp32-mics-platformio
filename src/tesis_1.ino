
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
// Global variable to track the latest timestamp we've used
static uint64_t lastUsedTimestamp = 0;
Adafruit_CCS811 ccs;
Adafruit_SSD1306 display(SSD_SCREEN_WIDTH, SSD_SCREEN_HEIGHT, &Wire, SSD_OLED_RESET);
const char* mqtt_server = MQTT_SERVER;
const char* mqtt_username = MQTT_USER;
const char* mqtt_password = MQTT_PASS;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
const char* identity = WIFI_IDENTITY;

// Add these globals near your other global variables
float lastCO2 = 0;
float lastTVOC = 0;
float lastCO = 0;
float lastNO2 = 0;
bool css811DataReady = false;
bool mics6814DataReady = false;
unsigned long lastCombinedDisplayUpdate = 0;

int mode = 1;

WiFiClient espClient;
PubSubClient client(espClient);

// Get the latest timestamp from offline data
uint64_t getLatestOfflineTimestamp() {
  uint64_t latestTimestamp = 0;
  
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    Serial.println("No offline data file exists");
    return 0;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, FILE_READ);
  if (!file) {
    Serial.println("Failed to open offline data file for reading timestamp");
    return 0;
  }
  
  // Get file size
  size_t fileSize = file.size();
  
  if (fileSize == 0) {
    Serial.println("Offline data file is empty");
    file.close();
    return 0;
  }
  
  // Strategy: Find the last newline character and read from there
  // Start from the end and read backwards until we find a newline
  long position = fileSize - 2; // Start before the possible last newline
  bool foundNewline = false;
  
  // Look for the last newline character
  while (position >= 0 && !foundNewline) {
    file.seek(position);
    char c = file.read();
    if (c == '\n') {
      foundNewline = true;
    } else {
      position--;
    }
  }
  
  // Position now points to the last newline (or -1 if no newline was found)
  // Move to the next character after the newline (or start of file if no newline)
  file.seek(position + 1);
  
  // Read the last line
  String lastLine = file.readStringUntil('\n');
  file.close();
  
  // Process the last line
  if (lastLine.length() > 0) {
    // Extract timestamp from the end of the line
    int spacePos = lastLine.lastIndexOf(' ');
    if (spacePos > 0 && spacePos < lastLine.length() - 1) {
      String timestampStr = lastLine.substring(spacePos + 1);
      latestTimestamp = strtoull(timestampStr.c_str(), NULL, 10);
      
      Serial.print("Latest offline timestamp (last line): ");
      Serial.println(String((uint32_t)(latestTimestamp / 1000000000)) + "." + String((uint32_t)(latestTimestamp % 1000000000)));
    } else {
      Serial.println("No timestamp found in last line");
    }
  } else {
    Serial.println("Last line is empty");
  }
  
  return latestTimestamp;
}

// Calculate timestamp difference in seconds
double calculateTimeDifference(uint64_t timestamp1, uint64_t timestamp2) {
  // Convert to seconds with decimal precision
  double t1 = (double)timestamp1 / 1000000000.0;
  double t2 = (double)timestamp2 / 1000000000.0;
  return t2 - t1;
}

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

// Function to clear offline data with visual feedback
void clearOfflineData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(10, 0);
  display.print("Clearing Offline Data");
  display.drawLine(0, 9, 128, 9, WHITE);
  display.display();
  
  // First check if the file exists
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    display.setCursor(0, 20);
    display.print("No offline data to clear!");
    display.display();
    delay(2000);
    return;
  }
  
  // Count records before deletion for feedback
  int recordCount = countOfflineRecords();
  
  display.setCursor(0, 20);
  display.printf("Records to clear: %d", recordCount);
  display.display();
  
  // Animation for deletion process
  display.setCursor(0, 30);
  display.print("Progress: ");
  display.drawRect(0, 40, 128, 10, WHITE);
  display.display();
  
  // Simulate progress with animation
  for (int i = 0; i <= 100; i += 10) {
    int barWidth = (i * 126) / 100;
    display.fillRect(2, 42, barWidth, 6, WHITE);
    display.setCursor(60, 30);
    display.printf("%d%%", i);
    display.display();
    delay(50);
  }
  
  // Actually delete the file
  bool success = SPIFFS.remove(OFFLINE_DATA_FILE);
  
  display.setCursor(0, 52);
  if (success) {
    display.print("Successfully cleared data!");
    Serial.println("Offline data cleared successfully");
  } else {
    display.print("Failed to clear data!");
    Serial.println("Failed to clear offline data");
  }
  display.display();
  delay(2000);
  
  // Report new storage status
  reportSPIFFSSpace();
}

// Function to check and report SPIFFS space usage
void reportSPIFFSSpace() {
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;
  
  float usedPercent = 100.0 * usedBytes / totalBytes;
  
  Serial.println("SPIFFS Space Usage:");
  Serial.print("Total: ");
  Serial.print(totalBytes);
  Serial.println(" bytes");
  Serial.print("Used: ");
  Serial.print(usedBytes);
  Serial.print(" bytes (");
  Serial.print(usedPercent, 1);
  Serial.println("%)");
  Serial.print("Free: ");
  Serial.print(freeBytes);
  Serial.print(" bytes (");
  Serial.print(100.0 - usedPercent, 1);
  Serial.println("%)");
  
  // If storage is getting full (over 80%), show a warning
  if (usedPercent > 80.0) {
    Serial.println("WARNING: SPIFFS storage is getting full!");
  }
}

// Initialize time from NTP server
void initTime() {
 bool ntpSuccess = false;
  
  // Only try NTP if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
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
      Serial.println("\nTime initialized from NTP!");
      ntpSuccess = true;
      timeInitialized = true;
      
      struct tm timeinfo;
      getLocalTime(&timeinfo);
      Serial.print("Current time: ");
      Serial.println(asctime(&timeinfo));
    } else {
      Serial.println("\nFailed to get time from NTP server");
    }
  } else {
    Serial.println("WiFi not connected, skipping NTP time sync");
  }

  // Always check for offline timestamps
  uint64_t latestOfflineTime = getLatestOfflineTimestamp();
  
  if (latestOfflineTime > 0) {
    if (ntpSuccess) {
      // Compare with NTP time if available
      struct timeval tv;
      gettimeofday(&tv, NULL);
      
      // Calculate current timestamp in our format
      uint64_t currentTimestamp = ((uint64_t)tv.tv_sec * 1000000000) + ((uint64_t)tv.tv_usec * 1000);
      
      // Check if offline timestamp is in the future compared to NTP time
      if (latestOfflineTime > currentTimestamp) {
        Serial.println("Offline timestamp is more recent than NTP time!");
        Serial.print("Difference (seconds): ");
        Serial.println(calculateTimeDifference(currentTimestamp, latestOfflineTime));
        
        // If offline timestamp is newer, use it to set system time
        // Extract seconds and microseconds
        uint64_t seconds = latestOfflineTime / 1000000000;
        uint64_t micros = (latestOfflineTime % 1000000000) / 1000;
        
        // Add a small increment (5 seconds) to ensure we're not reusing the exact same timestamp
        struct timeval newTime;
        newTime.tv_sec = seconds + 5;
        newTime.tv_usec = micros;
        
        if (settimeofday(&newTime, NULL) == 0) {
          Serial.println("System time updated with offline timestamp + 5 seconds");
          
          struct tm timeinfo;
          getLocalTime(&timeinfo);
          Serial.print("Updated time: ");
          Serial.println(asctime(&timeinfo));
        } else {
          Serial.println("Failed to update system time with offline timestamp");
          // Still use the offline timestamp as reference for our timestamps
          lastUsedTimestamp = latestOfflineTime + 5000000000; // Add 5 seconds in nanoseconds
        }
      } else {
        // NTP time is newer, but store the last offline timestamp for reference
        lastUsedTimestamp = latestOfflineTime;
      }
    } else {
      // No NTP time, use offline time
      Serial.println("Using recovered timestamp from offline data");
      
      // Extract seconds and microseconds
      uint64_t seconds = latestOfflineTime / 1000000000;
      uint64_t micros = (latestOfflineTime % 1000000000) / 1000;
      
      // Set a new time based on the latest offline timestamp plus 1 minute
      struct timeval tv;
      tv.tv_sec = seconds + 60; // Add 60 seconds
      tv.tv_usec = micros;
      
      // Set the system time
      if (settimeofday(&tv, NULL) == 0) {
        Serial.println("System time set from offline data (plus 1 minute)");
        timeInitialized = true;
        
        // Initialize lastUsedTimestamp with the offline timestamp plus 1 minute
        lastUsedTimestamp = latestOfflineTime + 60000000000; // 60 seconds in nanoseconds
        
        struct tm timeinfo;
        getLocalTime(&timeinfo);
        Serial.print("Current time set to: ");
        Serial.println(asctime(&timeinfo));
      } else {
        Serial.println("Failed to set system time from offline data");
        // Even if settimeofday fails, we can still use the timestamp for data recording
        timeInitialized = true; // Consider time initialized anyway
        lastUsedTimestamp = latestOfflineTime + 60000000000;
        Serial.println("Using offline timestamp as reference for data");
      }
    }
  } else if (!ntpSuccess) {
    Serial.println("No offline timestamps available and NTP failed");
    // In this case, we'll use millis() as fallback in saveOfflineData
  }
}

// Save data to SPIFFS when connection is unavailable
void saveOfflineData(const char* payload) {
  File file = SPIFFS.open(OFFLINE_DATA_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  
  // Add timestamp to the payload
  String timestampedPayload = String(payload);
  uint64_t currentTimestamp = 0;
  
  if (timeInitialized) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    
    // Calculate current timestamp
    currentTimestamp = ((uint64_t)tv.tv_sec * 1000000000) + ((uint64_t)tv.tv_usec * 1000);
    
    // If we have a previously used timestamp that's in the future compared to current time
    // (which can happen after power loss/restart), use that as a base and increment
    if (lastUsedTimestamp > 0 && lastUsedTimestamp > currentTimestamp) {
      // Use last timestamp + 1 second
      currentTimestamp = lastUsedTimestamp + 1000000000;
      Serial.println("Using incremented offline timestamp as reference");
    }
    
    // Format timestamp as a string
    char timestampStr[24]; // Large enough for uint64_t
    sprintf(timestampStr, "%lld%06lld000", currentTimestamp / 1000000000, (currentTimestamp % 1000000000) / 1000);
    
    timestampedPayload += " " + String(timestampStr);
    
    Serial.print("Full timestamp: ");
    Serial.println(timestampStr);
  } else {
    // Use millis() as fallback if time is not initialized
    unsigned long ms = millis();
    currentTimestamp = ms * 1000000ULL; // Convert to our timestamp format
    timestampedPayload += " " + String(ms) + "000000000";
  }
  
  // Update our last used timestamp
  lastUsedTimestamp = currentTimestamp;
  
  file.println(timestampedPayload);
  file.close();
  
  Serial.print("Saved offline data: ");
  Serial.println(timestampedPayload);

  // Report SPIFFS space usage after saving
  reportSPIFFSSpace();


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
  const size_t bufSize = 512;  // Process 512 bytes at a time
  char buf[bufSize];
  size_t bytesRead;
  
  while ((bytesRead = file.read((uint8_t*)buf, bufSize)) > 0) {
    for (size_t i = 0; i < bytesRead; i++) {
      if (buf[i] == '\n') {
        lineCount++;
      }
    }
  }
  
  // Check if the file doesn't end with a newline but has content
  if (file.size() > 0 && file.peek() != -1) {
    lineCount++;
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
  int totalRecords = countOfflineRecords();
  size_t fileSize = file.size();
  size_t processedBytes = 0;
  
  Serial.printf("Syncing %d offline records...\n", totalRecords);
  
  // Show sync progress screen if in mode 3 or 1
  if (mode == 3 || mode == 1) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(15, 0);
    display.print("Syncing Offline Data");
    display.drawLine(0, 9, 128, 9, WHITE);
    display.setCursor(0, 12);
    display.printf("Records: %d", totalRecords);
    display.setCursor(0, 22);
    display.print("Progress: 0%");
    display.drawRect(0, 32, 128, 10, WHITE);
    display.display();
  }
  
  // Process each line in the file
int recordNum = 0;
while (file.available()) {
  String line = file.readStringUntil('\n');
  line.trim(); // Remove any whitespace including leading/trailing spaces
  
  // Only process non-empty lines
  if (line.length() > 0) {
    recordNum++;
    processedBytes = file.position();
    
    // Update progress bar every few records
    if (recordNum % 5 == 0 || recordNum == totalRecords) {
      int progressPercent = (processedBytes * 100) / fileSize;
      int progressBarWidth = (progressPercent * 126) / 100;
      
      if (mode == 3 || mode == 1) {
        display.fillRect(0, 22, 128, 8, BLACK);
        display.setCursor(0, 22);
        display.printf("Progress: %d%%", progressPercent);
        display.fillRect(2, 34, progressBarWidth, 6, WHITE);
        
        display.setCursor(0, 44);
        display.printf("Synced: %d, Failed: %d", syncedCount, failedCount);
        display.display();
      }
    }
    
          // Check MQTT connection before publishing
    if (!client.connected() && WiFi.isConnected()) {
      Serial.println("MQTT disconnected, attempting to reconnect...");
      reconnectMQTT();
      
      // If reconnection failed, continue to next record but save this one
      if (!client.connected()) {
        failedCount++;
        tempFile.println(line);
        Serial.println("MQTT reconnection failed, saved record for later");
        continue;
      }
    }
    // Try to publish to MQTT with retry logic
    bool publishSuccess = false;
    int retryCount = 0;
    const int maxRetries = 2; // Number of retry attempts
    
    
    while (!publishSuccess && retryCount < maxRetries) {
      if (client.publish("egcs/egc-1", line.c_str())) {
        syncedCount++;
        Serial.print("Synced: ");
        Serial.println(line);
        publishSuccess = true;
      } else {
        retryCount++;
        if (retryCount < maxRetries) {
          Serial.print("Retry attempt ");
          
          Serial.print(retryCount);
          Serial.println(" for MQTT publish...");
          delay(500); // Wait a bit before retrying
        }
      }
    }
    
    // If all publishing attempts failed, save to temp file
    if (!publishSuccess) {
      failedCount++;
      tempFile.println(line);  // Keep the record for next attempt
      Serial.print("Failed to sync after retries: ");
      Serial.println(line);
    }
    
    // Small delay to avoid flooding the broker
    delay(10);
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
  
  // Final progress update
  if (mode == 3 || mode == 1) {
    display.fillRect(0, 22, 128, 8, BLACK);
    display.setCursor(0, 22);
    display.print("Progress: 100%");
    display.fillRect(2, 34, 126, 6, WHITE);
    display.setCursor(0, 44);
    display.printf("Synced: %d, Failed: %d", syncedCount, failedCount);
    display.setCursor(0, 54);
    display.print("Sync complete!");
    display.display();
    delay(1000); // Show final result for a second
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
    
    // Instead of clearing the whole display, just indicate reconnection attempt with a blinking WiFi status
    // Save current mode to restore proper display later
    int currentMode = mode;
    bool blinkState = true;
    
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);

    int maxAttempts = 10;
    int attempt = 0;
    
    while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts) {
      // Blink WiFi status indicator instead of clearing display
      if (currentMode == 1 && css811DataReady && mics6814DataReady) {
        // Keep the combined display but update just the WiFi status part
        display.fillRect(31, 33, 20, 8, BLACK); // Clear just the WiFi status area
        display.setCursor(31, 33);
        
        // Alternate between "..." and "   " for blinking effect
        if (blinkState) {
          display.print("...");
        } else {
          display.print("   ");
        }
        display.display();
        blinkState = !blinkState;
      } else if (currentMode == 3) {
        // If in offline records view, don't change anything
      } else {
        // For other modes or if data isn't ready, show a small indicator
        display.fillRect(0, 0, 8, 8, BLACK);
        if (blinkState) {
          display.fillRect(0, 0, 4, 4, WHITE);
        }
        display.display();
        blinkState = !blinkState;
      }
      
      delay(1000);
      Serial.print(".");
      attempt++;
    }
    
    // WiFi reconnection successful
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected to WiFi!");
      
      // Small visual indicator for successful reconnection
      if (currentMode == 1 && css811DataReady && mics6814DataReady) {
        display.fillRect(31, 33, 20, 8, BLACK);
        display.setCursor(31, 33);
        display.print("OK");
        display.display();
      }
      
      // Always try to update time when WiFi reconnects
      // This ensures we get accurate NTP time when available
      Serial.println("WiFi reconnected, updating time from NTP...");
      
      // Reset NTP time sync
      configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
      
      // Wait for time to be set from NTP
      time_t now = time(nullptr);
      int timeAttempts = 0;
      while (now < 8 * 3600 * 2 && timeAttempts < 5) {
        Serial.print(".");
        delay(500);
        now = time(nullptr);
        timeAttempts++;
      }
      
      if (now > 8 * 3600 * 2) {
        Serial.println("\nTime updated from NTP after reconnection!");
        timeInitialized = true;
        
        struct tm timeinfo;
        getLocalTime(&timeinfo);
        Serial.print("Updated time: ");
        Serial.println(asctime(&timeinfo));
        
        // Update timestamp for offline data
        struct timeval tv;
        gettimeofday(&tv, NULL);
        lastUsedTimestamp = ((uint64_t)tv.tv_sec * 1000000000) + ((uint64_t)tv.tv_usec * 1000);
        
        // Show visual indicator that time was updated
        for (int i = 0; i < 3; i++) {
          display.fillRect(120, 0, 8, 8, BLACK);
          if (i % 2 == 0) {
            display.fillRect(120, 0, 8, 8, WHITE);
          }
          display.display();
          delay(200);
        }
      } else {
        Serial.println("\nFailed to update time from NTP after reconnection");
      }
      
      // Force update the combined display with new time
      if (currentMode == 1 && css811DataReady && mics6814DataReady) {
        displayCombinedSensorData(lastCO2, lastTVOC, lastCO, lastNO2);
      }
      
      // Try to sync offline data if we have any
      int offlineCount = countOfflineRecords();
      if (offlineCount > 0 && client.connected()) {
        // Show a small indicator that sync is happening
        display.fillRect(112, 43, 16, 8, BLACK);
        display.setCursor(112, 43);
        display.print("↑");
        display.display();
        
        // Sync data
        syncOfflineData();
        
        // Update count on screen after sync
        if (currentMode == 1 && css811DataReady && mics6814DataReady) {
          displayCombinedSensorData(lastCO2, lastTVOC, lastCO, lastNO2);
        }
      }
    } else {
      // WiFi reconnection failed
      Serial.println("\nFailed to reconnect WiFi");
      
      if (currentMode == 1 && css811DataReady && mics6814DataReady) {
        display.fillRect(31, 33, 20, 8, BLACK);
        display.setCursor(31, 33);
        display.print("X");
        display.display();
      }
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

void monitorMICS(bool updateDisplay = true) {
  static unsigned long lastPublishTime = 0;
  unsigned long currentTime = millis();
  float COval, NO2val;
  COval = ppmToUgM3(CO);
  NO2val = ppmToUgM3(NO2);

  // Save the latest readings to global variables
  lastCO = COval;
  lastNO2 = NO2val;
  mics6814DataReady = true;

   // Only log detailed readings if displaying or on publish interval
  if (updateDisplay || currentTime - lastPublishTime >= 60000) {
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
    Serial.println(COval);
    
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
    Serial.println(NO2val);
    Serial.println("----------------------");
  }
  if(currentTime - lastPublishTime >= 60000) {
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
        lastPublishTime = currentTime;
      }
    } else {
      Serial.println("WiFi or MQTT not connected, saving offline");
      saveOfflineData(payload);
      lastPublishTime = currentTime;
    }
    
    // Try to sync offline data when we have a connection
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      syncOfflineData();
    }
  }

    // Combined display: check if both sensors have data and it's time to update
  if(updateDisplay && mode == 1 && mics6814DataReady && css811DataReady) {
    if (millis() - lastCombinedDisplayUpdate > 1000) { // Update display once per second
      displayCombinedSensorData(lastCO2, lastTVOC, lastCO, lastNO2);
      lastCombinedDisplayUpdate = millis();
    }
  }
}

void monitorCSS811(bool updateDisplay = true) {
  static unsigned long lastPublishTime = 0;
  unsigned long currentTime = millis();
  float CO2val, TVOCval;
  if(ccs.available()){
      if(!ccs.readData()){
        CO2val = ccs.geteCO2();
        TVOCval = ccs.getTVOC();

        // Save the latest readings to global variables
        lastCO2 = CO2val;
        lastTVOC = TVOCval;
        css811DataReady = true;

        // Only log detailed readings if displaying or on publish interval
        if (updateDisplay || currentTime - lastPublishTime >= 60000) {
          Serial.println("----------------------");
          Serial.println("CCS811 Sensor Readings:");
          Serial.print("CO2: ");
          Serial.print(CO2val);
          Serial.println(" ppm");
          Serial.print("TVOC: ");
          Serial.print(TVOCval);
          Serial.println(" ppb");
          Serial.println("----------------------");
        }

        if(currentTime - lastPublishTime >= 60000) {
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
              lastPublishTime = currentTime;
            }
          } else {
            Serial.println("WiFi or MQTT not connected, saving offline");
            saveOfflineData(payload);
            lastPublishTime = currentTime;
          }
          
          // Try to sync offline data when we have a connection
          if (WiFi.status() == WL_CONNECTED && client.connected()) {
            syncOfflineData();
          }
        }

        // Combined display: check if both sensors have data and it's time to update
        if(updateDisplay && mode == 1 && mics6814DataReady && css811DataReady) {
          if (millis() - lastCombinedDisplayUpdate > 1000) { // Update display once per second
            displayCombinedSensorData(lastCO2, lastTVOC, lastCO, lastNO2);
            lastCombinedDisplayUpdate = millis();
          }
        }
    
        } else {
          // Error handling
        if (updateDisplay) {
          static unsigned long lastErrorTime = 0;
          if (millis() - lastErrorTime > 5000) { // Only show error every 5 seconds
            lastErrorTime = millis();
            Serial.println("CCS811 read error!");
            
            // Update display with error but don't block
            display.clearDisplay();
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.print("CCS811 Error");
            display.setCursor(0, 20);
            display.print("Retrying...");
            display.display();
          }
      } else {
        // Minimal error logging in background mode
        static unsigned long lastErrorTime = 0;
        if (millis() - lastErrorTime > 30000) { // Only log error every 30 seconds
          lastErrorTime = millis();
          Serial.println("CCS811 read error (Background mode)!");
        }
      }
    }
  }
}
// Add this function to display offline records info
void displayOfflineRecords() {
  int recordCount = countOfflineRecords();
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  float usedPercent = 100.0 * usedBytes / totalBytes;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(13, 0);
  display.print("Offline Data Status");
  display.drawLine(0, 9, 128, 9, WHITE);
  
  display.setCursor(0, 12);
  display.print("Records: ");
  display.print(recordCount);
  
  // Show storage usage
  display.setCursor(0, 22);
  display.print("Used: ");
  display.print(usedBytes);
  display.print("/");
  display.print(totalBytes);
  
  display.setCursor(0, 32);
  display.print("Percent: ");
  display.print(usedPercent, 1);
  display.print("%");
  
  // Show storage bar
  int barWidth = 100;
  int barHeight = 6;
  int filledWidth = (usedPercent / 100.0) * barWidth;
  
  display.drawRect(12, 42, barWidth, barHeight, WHITE);
  display.fillRect(12, 42, filledWidth, barHeight, WHITE);
  
  // Show the last timestamp if records exist
  if (recordCount > 0) {
    uint64_t latestTimestamp = getLatestOfflineTimestamp();
    
    if (latestTimestamp > 0) {
      display.setCursor(0, 52);
      display.print("Last: ");
      
      // Convert timestamp to human readable time
      time_t seconds = latestTimestamp / 1000000000;
      struct tm timeinfo;
      localtime_r(&seconds, &timeinfo);
      
      char timeStr[20];
      strftime(timeStr, sizeof(timeStr), "%m-%d %H:%M:%S", &timeinfo);
      
      display.setCursor(35, 52);
      display.print(timeStr);
    }
  }
  
  display.display();
}

// Add this function to create a transition animation when switching modes
void animateModeTransition(int fromMode, int toMode) {
  // Save current display buffer
  display.clearDisplay();
  
  // Animation parameters
  const int steps = 10;
  const int duration = 300; // Total animation duration in ms
  const int delayPerStep = duration / steps;
  
  // Different animation styles based on direction of mode change
  if (fromMode < toMode) {
    // Sliding animation from right to left
    for (int step = 0; step <= steps; step++) {
      display.clearDisplay();
      
      // Draw mode indicator
      display.setTextSize(2);
      display.setCursor(25, 25);
      
      // Calculate position for sliding text
      int pos = map(step, 0, steps, display.width(), 25);
      
      display.setCursor(pos, 25);
      display.print("MODE ");
      display.print(toMode);
      
      display.display();
      delay(delayPerStep);
    }
  } else if (fromMode > toMode) {
    // Fading/blinking animation
    for (int step = 0; step <= steps; step++) {
      display.clearDisplay();
      
      // Draw mode indicator with alternating visibility
      if ((step % 2) == 0 || step > steps-3) {
        display.setTextSize(2);
        display.setCursor(25, 25);
        display.print("MODE ");
        display.print(toMode);
      }
      
      display.display();
      delay(delayPerStep);
    }
  } else {
    // Same mode, just visual feedback
    for (int step = 0; step <= steps/2; step++) {
      display.clearDisplay();
      
      // Pulse effect with size
      int textSize = (step < steps/4) ? 2 : 1;
      display.setTextSize(textSize);
      int yPos = (textSize == 2) ? 25 : 30;
      
      // Calculate position to keep text centered
      int xPos = (textSize == 2) ? 25 : 35;
      
      display.setCursor(xPos, yPos);
      display.print("MODE ");
      display.print(toMode);
      
      display.display();
      delay(delayPerStep*2);
    }
  }
  
  // Final mode display
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(25, 25);
  display.print("MODE ");
  display.print(toMode);
  
  // Add mode description
  display.setTextSize(1);
  display.setCursor(20, 50);
  
  // Update mode descriptions
  switch (toMode) {
    case 1:
      display.print("Combined Sensors");
      break;
    case 2:
      display.print("System Status");  // Repurpose mode 2 if needed
      break;
    case 3:
      display.print("Offline Data Status");
      break;
  }
  
  display.display();
  delay(500); // Show mode for half a second
}

// Add this new combined monitoring display function
void displayCombinedSensorData(float CO2val, float TVOCval, float COval, float NO2val) {
   display.clearDisplay();
  
  // Header
  display.setTextSize(1);
  display.setCursor(15, 0);
  display.print("Air Quality Monitor");
  display.drawLine(0, 8, 128, 8, WHITE);
  
  // Left side - first row
  display.setCursor(0, 11);
  display.print("CO2:");
  display.setCursor(26, 11);
  display.print(CO2val, 0); // No decimal points to save space
  display.print(" ppm");
  
  // Right side - first row
  display.setCursor(68, 11);
  display.print("CO:");
  display.setCursor(86, 11);
  display.print(COval, 1); // 1 decimal point
  
  // Left side - second row
  display.setCursor(0, 21);
  display.print("TVOC:");
  display.setCursor(32, 21);
  display.print(TVOCval, 0); // No decimal points
  display.print(" ppb");
  
  // Right side - second row
  display.setCursor(68, 21);
  display.print("NO2:");
  display.setCursor(92, 21);
  display.print(NO2val, 1); // 1 decimal point
  
  // Status section
  display.drawLine(0, 30, 128, 30, WHITE);
  
  // Status elements in a more compact layout
  display.setCursor(0, 33);
  display.print("WiFi:");
  display.setCursor(31, 33);
  display.print(WiFi.status() == WL_CONNECTED ? "OK" : "X");
  
  display.setCursor(66, 33);
  display.print("MQTT:");
  display.setCursor(97, 33);
  display.print(client.connected() ? "OK" : "X");
  
  // Offline storage bar
  int offlineCount = countOfflineRecords();
  display.setCursor(0, 43);
  display.print("Records:");
  display.setCursor(48, 43);
  display.print(offlineCount);
  
  // Get and display the current time instead of storage percentage
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm timeinfo;
  localtime_r(&tv.tv_sec, &timeinfo);
  
  char dateStr[11]; // YYYY-MM-DD
  char timeStr[9];  // HH:MM:SS
  
  strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &timeinfo);
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
  
  // Show date and time
  display.setCursor(0, 53);
  display.print(dateStr);
  display.setCursor(65, 53);
  display.print(timeStr);
  
  display.display();
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
  // clearOfflineData();
  // Step 1: Initialize SPIFFS
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Initializing SPIFFS...");
  display.display();
  initSPIFFS();
  display.setCursor(0, 10);
  display.print("SPIFFS initialized!");
  display.display();
  delay(1000);

   // Step 2: Connect to WiFi
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Connecting to WiFi...");
  display.display();
  connectToWiFi();
  
  // Step 3: Initialize time
  display.clearDisplay();
  display.setCursor(0, 0);
  if (WiFi.status() == WL_CONNECTED) {
    display.print("WiFi connected!");
    display.setCursor(0, 10);
    display.print("Setting up time via NTP...");
  } else {
    display.print("WiFi not connected!");
    display.setCursor(0, 10);
    display.print("Setting up local time...");
  }
  display.display();
  
  // Always call initTime() - it will handle both WiFi and non-WiFi cases
  initTime();

  // Show time status
  display.setCursor(0, 20);
  if (timeInitialized) {
    display.print("Time setup done!");
    
    // Get timestamp and display its source
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm timeinfo;
    localtime_r(&tv.tv_sec, &timeinfo);
    
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    
    display.setCursor(0, 30);
    display.print("Time: ");
    display.print(timeStr);
    
    display.setCursor(0, 40);
    display.print("Source: ");
    display.print(WiFi.status() == WL_CONNECTED ? "NTP" : "Offline");
  } else {
    display.setCursor(0, 30);
    display.print("No time source available");
    display.setCursor(0, 40);
    display.print("Using fallback timestamps");
  }
  display.display();
  delay(1500);

  // Step 4: Initialize CCS811 Sensor
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Initializing CCS811...");
  display.display();
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor!");
    display.setCursor(0, 10);
    display.print("CCS811 failed!");
    display.display();
    while (1);
  }
  while (!ccs.available());
  display.setCursor(0, 10);
  display.print("CCS811 initialized!");
  display.display();
  delay(1000);

  // Step 5: Initialize MICS6814 Sensor
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Initializing MICS6814...");
  display.display();
  initMICS(NH3PIN, COPIN, OXPIN, MICS_CALIBRATION_SECONDS, MICS_CALIBRATION_DELTA);
  // clearMICSCalibration();
  calibrateMICS();
  display.setCursor(0, 10);
  display.print("MICS6814 initialized!");
  display.display();
  delay(1000);

  // // Step 6: Set up MQTT client
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Setting up MQTT...");
  display.display();
  client.setServer(mqtt_server, MQTT_PORT);
  display.setCursor(0, 10);
  display.print("MQTT setup done!");

  pinMode(BUTTON_PIN, INPUT);
    // Step 7: Set up button
  display.setCursor(0, 20);
  display.print("Button setup done!");

  // Show offline records count if any
  int offlineCount = countOfflineRecords();
  if (offlineCount > 0) {
    display.setCursor(0, 30);
    display.print("Offline records: ");
    display.print(offlineCount);
  }
  display.display();
  delay(1500);

  // Show startup complete screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(15, 10);
  display.print("System Ready");
  display.setCursor(20, 25);
  display.print("Air Quality");
  display.setCursor(28, 35);
  display.print("Monitor");
  display.setCursor(15, 50);
  display.print("Starting...");
  display.display();
  delay(1000);
}

void loop() {
  static unsigned long buttonPressStartTime = 0;
  static bool buttonPressed = false;
  static unsigned long lastPublishTime = 0;
  static unsigned long lastSyncAttempt = 0;
  static unsigned long modeChangeTime = 0;

  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStartTime = millis();
    } else {
      unsigned long pressDuration = millis() - buttonPressStartTime;
      
      // Short press (0.5-1 second) - switch between mode 1 and 2
      if (pressDuration >= 500 && pressDuration < 1000) {
        int newMode = mode;
        
        if (mode == 1) {
          newMode = 3;  // Go directly to offline data status
        } else if (mode == 3) {
          newMode = 1;  // Back to combined sensor display
        }
        
        // If mode is changing, show animation
        if (newMode != mode) {
          animateModeTransition(mode, newMode);
          mode = newMode;
          modeChangeTime = millis();
        }
        
        buttonPressed = false; // Reset the button pressed state
      } else if (pressDuration >= 1000 && pressDuration < 3000) {
        if (mode != 3) {
          animateModeTransition(mode, 3);
          mode = 3;
          modeChangeTime = millis();
        }
        buttonPressed = false; // Reset the button pressed state
      }
    }
  } else {
    buttonPressed = false;
  }

  // Auto-return from mode 3 to mode 1 after 10 seconds
  if (mode == 3 && millis() - modeChangeTime > 10000) {
    animateModeTransition(3, 1);
    mode = 1;
  }

  // Check and reconnect MQTT if needed
  if(!client.connected() && WiFi.status() == WL_CONNECTED) {
    reconnectMQTT();
  }

    // Sync offline data periodically if connected
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    if (millis() - lastSyncAttempt > 60000) { // Try every minute
      lastSyncAttempt = millis();
           // Try to sync offline data if we have any
      int offlineCount = countOfflineRecords();
      if (offlineCount > 0) {
        display.setCursor(0, 50);
        display.print("Offline records: ");
        display.print(offlineCount);
        display.setCursor(0, 60);
        display.print("Syncing...");
        display.display();
        syncOfflineData();
      }
      
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

    // Always monitor sensors for data collection
  // but only update display in modes 1 and 2
  bool updateDisplay = (mode == 1 || mode == 2);
  monitorMICS(updateDisplay);
  monitorCSS811(updateDisplay);
  // Display based on current mode
  if (mode == 3) {
    // Mode 3: Display offline records info
    static unsigned long lastOfflineRecordUpdate = 0;
    if (millis() - lastOfflineRecordUpdate > 2000) { // Update every 2 seconds
      lastOfflineRecordUpdate = millis();
      displayOfflineRecords();
    }
  }
  delay(100);
}


