#include "MICS6814.h"
#include <EEPROM.h>

#define EEPROM_NH3_BASE_ADDR 0
#define EEPROM_RED_BASE_ADDR 2
#define EEPROM_OX_BASE_ADDR 4
#define EEPROM_CALIBRATION_FLAG_ADDR 6
#define CALIBRATION_FLAG_VALUE 0xAA


uint16_t NH3baseR;
uint16_t REDbaseR;
uint16_t OXbaseR;

uint8_t NH3PIN;
uint8_t COPIN;
uint8_t OXPIN;
uint8_t MICS_CALIBRATION_SECONDS;
uint8_t MICS_CALIBRATION_DELTA;

void saveBaseResistancesToEEPROM() {
    EEPROM.put(EEPROM_NH3_BASE_ADDR, NH3baseR);
    EEPROM.put(EEPROM_RED_BASE_ADDR, REDbaseR);
    EEPROM.put(EEPROM_OX_BASE_ADDR, OXbaseR);
    EEPROM.commit();
}

void loadBaseResistancesFromEEPROM() {
    EEPROM.get(EEPROM_NH3_BASE_ADDR, NH3baseR);
    EEPROM.get(EEPROM_RED_BASE_ADDR, REDbaseR);
    EEPROM.get(EEPROM_OX_BASE_ADDR, OXbaseR);
}

bool isCalibrated() {
    uint8_t flag;
    EEPROM.get(EEPROM_CALIBRATION_FLAG_ADDR, flag);
    return flag == CALIBRATION_FLAG_VALUE;
}

void initMICS(uint8_t nh3Pin, uint8_t coPin, uint8_t oxPin, uint8_t calibrationSeconds, uint8_t calibrationDelta) {
  NH3PIN = nh3Pin;
  COPIN = coPin;
  OXPIN = oxPin;
  MICS_CALIBRATION_SECONDS = calibrationSeconds;
  MICS_CALIBRATION_DELTA = calibrationDelta;
  EEPROM.begin(512);
  loadBaseResistancesFromEEPROM();
}

void calibrateMICS() {
  if (isCalibrated()) {
    Serial.println("Sensor already calibrated. Skipping calibration.");
    return;
  }
  // Continuously measure the resistance,
  // storing the last N measurements in a circular buffer.
  // Calculate the floating average of the last seconds.
  // If the current measurement is close to the average stop.

  // Seconds to keep stable for successful calibration
  // (Keeps smaller than 64 to prevent overflows)
  uint8_t seconds = 10;
  // Allowed delta for the average from the current value
  uint8_t delta = 4;

  // Circular buffer for the measurements
  uint16_t bufferNH3[seconds];
  uint16_t bufferRED[seconds];
  uint16_t bufferOX[seconds];
  // Pointers for the next element in the buffer
  uint8_t pntrNH3 = 0;
  uint8_t pntrRED = 0;
  uint8_t pntrOX = 0;
  // Current floating sum in the buffer
  uint16_t fltSumNH3 = 0;
  uint16_t fltSumRED = 0;
  uint16_t fltSumOX = 0;

  // Current measurements;
  uint16_t curNH3;
  uint16_t curRED;
  uint16_t curOX;

  // Flag to see if the channels are stable
  bool NH3stable = false;
  bool REDstable = false;
  bool OXstable = false;

  // Initialize buffer
  for (int i = 0; i < seconds; ++i) {
    bufferNH3[i] = 0;
    bufferRED[i] = 0;
    bufferOX[i] = 0;
  }

  do {
    // Wait a second
    delay(1000);
    Serial.print(".");
    // Read new resistances
    unsigned long rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(NH3PIN);
    }
    curNH3 = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(COPIN);
    }
    curRED = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(OXPIN);
    }
    curOX = rs/3;

    // Update floating sum by subtracting value
    // about to be overwritten and adding the new value.
    fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
    fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
    fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

    // Store new measurement in buffer
    bufferNH3[pntrNH3] = curNH3;
    bufferRED[pntrRED] = curRED;
    bufferOX[pntrOX] = curOX;

    // Determine new state of flags
    NH3stable = abs(fltSumNH3 / seconds - curNH3) < delta;
    REDstable = abs(fltSumRED / seconds - curRED) < delta;
    OXstable = abs(fltSumOX / seconds - curOX) < delta;

    // Advance buffer pointer
    pntrNH3 = (pntrNH3 + 1) % seconds ;
    pntrRED = (pntrRED + 1) % seconds;
    pntrOX = (pntrOX + 1) % seconds;

    //Mikä kestää?
    if(!NH3stable) {
      Serial.print("(NH3:");
      Serial.print(abs(fltSumNH3 / seconds - curNH3));
      Serial.print(")");
    }
    if(!REDstable) {
      Serial.print("(RED:");
      Serial.print(abs(fltSumNH3 / seconds - curRED));
      Serial.print(")");
    }
    if(!OXstable) {
      Serial.print("(OX:");
      Serial.print(abs(fltSumNH3 / seconds - curOX));
      Serial.print(")");
    }

  } while (!NH3stable || !REDstable || !OXstable);

  NH3baseR = fltSumNH3 / seconds;
  REDbaseR = fltSumRED / seconds;
  OXbaseR = fltSumOX / seconds;

  // Store new base resistance values in EEPROM
  saveBaseResistancesToEEPROM();
}

void calibrateMICSV2(){
    if (isCalibrated()) {
        Serial.println("Sensor already calibrated. Skipping calibration.");
        return;
    }
   // Circular buffer for the measurements
    uint16_t bufferNH3[MICS_CALIBRATION_SECONDS];
    uint16_t bufferRED[MICS_CALIBRATION_SECONDS];
    uint16_t bufferOX[MICS_CALIBRATION_SECONDS];

    // Pointers for the next element in the buffer
    uint8_t pntrNH3 = 0, pntrRED = 0, pntrOX = 0;

    // Current floating sum in the buffer
    uint16_t fltSumNH3 = 0, fltSumRED = 0, fltSumOX = 0;

    // Current measurements;
    uint16_t curNH3, curRED, curOX;

    bool micsOK = false;

    // Flag to see if the channels are stable
    bool NH3Stable = false;
    bool REDStable = false;
    bool OXStable = false;

     // Initialize buffer
    for (int i = 0; i < MICS_CALIBRATION_SECONDS; ++i) {
        bufferNH3[i] = 0;
        bufferRED[i] = 0;
        bufferOX[i] = 0;
    }

       do {
        // Wait a second
        delay(1000);
     // Read new resistance for NH3
        unsigned long rs = 0;
        delay(50);
        for (int i = 0; i < 3; i++) {
            delay(1);
            rs += analogRead(NH3PIN);
        }
        curNH3 = rs/3;
         // Read new resistance for CO
        rs = 0;
        delay(50);
        for (int i = 0; i < 3; i++) {
            delay(1);
            rs += analogRead(COPIN);
        }
        curRED = rs/3;

        // Read new resistance for NO2
        rs = 0;
        delay(50);
        for (int i = 0; i < 3; i++) {
            delay(1);
            rs += analogRead(OXPIN);
        }
        curOX = rs/3;

         // Update floating sum by subtracting value about to be overwritten and adding the new value.
        fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
        fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
        fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

        // Store new measurement in buffer
        bufferNH3[pntrNH3] = curNH3;
        bufferRED[pntrRED] = curRED;
        bufferOX[pntrOX] = curOX;

        // Determine new state of flags
        NH3Stable = abs(fltSumNH3 / MICS_CALIBRATION_SECONDS - curNH3) < MICS_CALIBRATION_DELTA;
        REDStable = abs(fltSumRED / MICS_CALIBRATION_SECONDS - curRED) < MICS_CALIBRATION_DELTA;
        OXStable = abs(fltSumOX / MICS_CALIBRATION_SECONDS - curOX) < MICS_CALIBRATION_DELTA;
        micsOK = NH3Stable && REDStable && OXStable;
         // Advance buffer pointer
        pntrNH3 = (pntrNH3 + 1) % MICS_CALIBRATION_SECONDS ;
        pntrRED = (pntrRED + 1) % MICS_CALIBRATION_SECONDS;
        pntrOX = (pntrOX + 1) % MICS_CALIBRATION_SECONDS;
       } while(!micsOK);
        Serial.println("DONE!");

      NH3baseR = fltSumNH3 / MICS_CALIBRATION_SECONDS;
      REDbaseR = fltSumRED / MICS_CALIBRATION_SECONDS;
      OXbaseR = fltSumOX / MICS_CALIBRATION_SECONDS;

      Serial.print("NH3 Base: ");
      Serial.print(NH3baseR);
      Serial.print("RED Base: ");
      Serial.print(REDbaseR);
      Serial.print("OX Base: ");
      Serial.print(OXbaseR);
       
    saveBaseResistancesToEEPROM();
}

void calibrateMICSV3() {
    if (isCalibrated()) {
        Serial.println("Sensor already calibrated. Skipping calibration.");
        return;
    }
  uint8_t seconds = 10;
  uint8_t delta = 3; // Adjust delta based on your ADC range and requirements

  uint16_t bufferNH3[seconds] = {0};
  uint16_t bufferRED[seconds] = {0};
  uint16_t bufferOX[seconds] = {0};

  uint8_t pntrNH3 = 0, pntrRED = 0, pntrOX = 0;
  uint16_t fltSumNH3 = 0, fltSumRED = 0, fltSumOX = 0;

  uint16_t curNH3, curRED, curOX;
  bool NH3stable = false, REDstable = false, OXstable = false;

  // Pre-fill buffers to prevent false stability detection
  for (int i = 0; i < seconds; ++i) {
    curNH3 = analogRead(NH3PIN);
    curRED = analogRead(COPIN);
    curOX = analogRead(OXPIN);
    bufferNH3[i] = curNH3;
    bufferRED[i] = curRED;
    bufferOX[i] = curOX;
    fltSumNH3 += curNH3;
    fltSumRED += curRED;
    fltSumOX += curOX;
    delay(50); // Allow sensor to stabilize between readings
  }

  do {
    delay(1000);
    Serial.print(".");

    // Read new resistances (average 3 samples per channel for stability)
    curNH3 = 0;
    curRED = 0;
    curOX = 0;
    for (int i = 0; i < 3; i++) {
      curNH3 += analogRead(NH3PIN);
      curRED += analogRead(COPIN);
      curOX += analogRead(OXPIN);
      delay(1);
    }
    curNH3 /= 3;
    curRED /= 3;
    curOX /= 3;

    // Update floating sums
    fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
    fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
    fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

    // Update buffers
    bufferNH3[pntrNH3] = curNH3;
    bufferRED[pntrRED] = curRED;
    bufferOX[pntrOX] = curOX;

    // Check stability
    NH3stable = abs(fltSumNH3 / seconds - curNH3) < delta;
    REDstable = abs(fltSumRED / seconds - curRED) < delta;
    OXstable = abs(fltSumOX / seconds - curOX) < delta;

    // Advance buffer pointers
    pntrNH3 = (pntrNH3 + 1) % seconds;
    pntrRED = (pntrRED + 1) % seconds;
    pntrOX = (pntrOX + 1) % seconds;

    // Debug stability deltas
    if (!NH3stable) {
      Serial.print("(NH3:");
      Serial.print(abs(fltSumNH3 / seconds - curNH3));
      Serial.print(")");
    }
    if (!REDstable) {
      Serial.print("(RED:");
      Serial.print(abs(fltSumRED / seconds - curRED));
      Serial.print(")");
    }
    if (!OXstable) {
      Serial.print("(OX:");
      Serial.print(abs(fltSumOX / seconds - curOX));
      Serial.print(")");
    }

  } while (!NH3stable || !REDstable || !OXstable);

  NH3baseR = fltSumNH3 / seconds;
  REDbaseR = fltSumRED / seconds;
  OXbaseR = fltSumOX / seconds;

  // Store base resistance values in EEPROM (add EEPROM write logic here)
   saveBaseResistancesToEEPROM();
}

uint16_t getResistance(channel_t channel) {
      unsigned long rs = 0;
      int counter = 0;

  switch (channel) {
    case CH_NH3:
      for(int i = 0; i < 100; i++) {
        rs += analogRead(NH3PIN);
        counter++;
        delay(2);
      }
      return rs/counter;
    case CH_RED:
      for(int i = 0; i < 100; i++) {
        rs += analogRead(COPIN);
        counter++;
        delay(2);
      }
      return rs/counter;
    case CH_OX:      
      for(int i = 0; i < 100; i++) {
        rs += analogRead(OXPIN);
        counter++;
        delay(2);
      }
      return rs/counter;
  }

  return 0;
}

uint16_t getBaseResistance(channel_t channel) {
  /* if (1 == __version) {
     // Version 1 can query every channel independently
     // Reply is 4 bytes long with relevant data in second and third byte
     switch (channel) {
       case CH_NH3:
         return getRuntimeData(CMD_V1_GET_R0_NH3, 4, 1);
       case CH_RED:
         return getRuntimeData(CMD_V1_GET_R0_RED, 4, 1);
       case CH_OX:
         return getRuntimeData(CMD_V1_GET_R0_OX, 4, 1);
     }
    }
    if (2 == __version) {
     // Version 2 uses the same command every time, but different offsets*/
     switch (channel) {
       case CH_NH3:
         return NH3baseR;
       case CH_RED:
         return REDbaseR;
       case CH_OX:
         return OXbaseR;
     }
  //  }
  
  return 0;
}

float getCurrentRatio(channel_t channel) {
  float baseResistance = (float) getBaseResistance(channel);
  float resistance = (float) getResistance(channel);

  return resistance / baseResistance * (4095.0 - baseResistance) / (4095.0 - resistance);
  

  return -1.0;
}

float measureMICS(gas_t gas) {
  float ratio;
  float c = 0; //ppm
  
  switch (gas) {
    case CO:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.179) * 4.385;
      break;
    case NO2:
      ratio = getCurrentRatio(CH_OX);
      c = pow(ratio, 1.007) / 6.855;
      break;
    case NH3:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -1.67) / 1.47;
      break;
    case C3H8:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -2.518) * 570.164;
      break;
    case C4H10:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -2.138) * 398.107;
      break;
    case CH4:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -4.363) * 630.957;
      break;
    case H2:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.8) * 0.73;
      break;
    case C2H5OH:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.552) * 1.622;
      break;
  }

  return isnan(c) ? -1 : c;
}

float ppmToUgM3(gas_t gas) {
  float ppm;
  float ugm3;
  float conversionFactor = 40.9;
  // ug/m3 = 40.9 * ppm * molecular weight
  // WITH TEMPERATURE 25 celcius & PRESSURE 1atm
  switch (gas) {
    case CO:
      ppm = measureMICS(CO);
      ugm3 = ppm * 28.01 * conversionFactor;
      break;
    case NO2:
      ppm = measureMICS(NO2);
      ugm3 = ppm * 46.01 * conversionFactor;
      break;
    case NH3:
      ppm = measureMICS(NH3);
      ugm3 = ppm * 17.03 * conversionFactor;
      break;
    case C3H8:
      ppm = measureMICS(C3H8);
      ugm3 = ppm * 44.1 * conversionFactor;
      break;
    case C4H10:
      ppm = measureMICS(C4H10);
      ugm3 = ppm * 58.12 * conversionFactor;
      break;
    case CH4:
      ppm = measureMICS(CH4);
      ugm3 = ppm * 16.04 * conversionFactor;
      break;
    case H2:
      ppm = measureMICS(H2);
      ugm3 = ppm * 2.02 * conversionFactor;
      break;
    case C2H5OH:
      ppm = measureMICS(C2H5OH);
      ugm3 = ppm * 46.07 * conversionFactor;
      break;
    default:
      ugm3 = -1; // Invalid gas type
      break;
  }

  return isnan(ugm3) ? -1 : ugm3;
}
