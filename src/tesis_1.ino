
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

Adafruit_CCS811 ccs;
Adafruit_SSD1306 display(SSD_SCREEN_WIDTH, SSD_SCREEN_HEIGHT, &Wire, SSD_OLED_RESET);
const char* mqtt_server = MQTT_SERVER;
const char* mqtt_username = MQTT_USER;
const char* mqtt_password = MQTT_PASS;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
int mode = 1;

WiFiClient espClient;
PubSubClient client(espClient);

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
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
    if (client.publish("egcs/egc-1", payload)) {
      Serial.println("Publish successful");
      lastPublishTime = currentTime;
    } else {
      Serial.println("Publish failed");
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
          if (client.publish("egcs/egc-1", payload)) {
            Serial.println("Publish successful");
            lastPublishTime = currentTime;
          } else {
            Serial.println("Publish failed");
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

  // Step 1: Connect to WiFi
  display.setCursor(0, 0);
  display.print("Connecting to WiFi...");
  display.display();
  connectToWiFi();
  display.setCursor(0, 10);
  display.print("WiFi connected!");
  display.display();
  delay(500);

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
  if(!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  monitorMICS();
  monitorCSS811();
  delay(1000);
}


