#ifndef MICS6814_H
#define MICS6814_H

#include <Arduino.h>

enum channel {
  CH_NH3, CH_RED, CH_OX
};
typedef enum channel channel_t;

enum gas {
  CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH
};
typedef enum gas gas_t;

void initMICS(uint8_t nh3Pin, uint8_t coPin, uint8_t oxPin, uint8_t calibrationSeconds, uint8_t calibrationDelta);
void calibrateMICS();
void calibrateMICSV2();
void calibrateMICSV3();
uint16_t getResistance(channel_t channel);
uint16_t getBaseResistance(channel_t channel);
float getCurrentRatio(channel_t channel);
float measureMICS(gas_t gas);
float ppmToUgM3(gas_t gas);

#endif