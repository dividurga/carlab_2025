#include "compass.h"
#include <EEPROM.h>
#include "qmc6310.h"
QMC6310 _compass;

int16_t heading;
int calibrationData[6];
#if (AVERAGE_FILTER)
int16_t filterBuffer[AVERAGE_FILTER_SIZE];
uint8_t filterBufferIndex = 0;
int32_t filterBufferSum = 0;
#endif
uint32_t calibrate_c;
uint32_t calibrate_t;
bool calibrateChanged;
bool calibrateDone = true;


/**
 * @brief Clear value of compass Calibration
 * 
 */
void compassClearCalibration() {
  for (uint8_t i = 0; i < 6; i++) {
    calibrationData[i] = 0;
    EEPROM.write(EEPROM_CALIBRATION_ADDRESS + 2 * i, 0);
    EEPROM.write(EEPROM_CALIBRATION_ADDRESS + 2 * i + 1, 0);
  }
  _compass.clearCalibration();
}

/**
 * @brief Save value of compass Calibration
 * 
 */
void compassSaveCalibration() {
  // Serial.println("Saving calibration data...");
  for (int i = 0; i < 6; i++) {
    uint16_t data = calibrationData[i] + 0xFF;
    uint8_t highByte = data >> 8;
    uint8_t lowByte = data & 0xFF;
    EEPROM.write(EEPROM_CALIBRATION_ADDRESS + 2 * i, highByte);
    EEPROM.write(EEPROM_CALIBRATION_ADDRESS + 2 * i + 1, lowByte);
  }
  _compass.setCalibration(
    calibrationData[0], 
    calibrationData[1], 
    calibrationData[2], 
    calibrationData[3], 
    calibrationData[4], 
    calibrationData[5]
  );
}

/**
 * @brief Read value of compass Calibration
 * 
 */
void compassReadCalibration() {
  uint8_t highByte, lowByte;
  // Serial.println("Reading calibrationData value from EEPROM");
  for (uint8_t i = 0; i < 6; i++) {
    highByte = EEPROM.read(EEPROM_CALIBRATION_ADDRESS + 2 * i);
    lowByte = EEPROM.read(EEPROM_CALIBRATION_ADDRESS + 2 * i + 1);
    long data = (highByte << 8) | lowByte;
    calibrationData[i] = data - 0xFF;
  }

  _compass.setCalibration(
    calibrationData[0],
    calibrationData[1],
    calibrationData[2],
    calibrationData[3],
    calibrationData[4],
    calibrationData[5]
  );
}


/**
 * @brief Is compass Calibrate Done ?
 * 
 * @return  - true 
 *          - false 
 *         
 */
bool compassCalibrateDone() {
  return calibrateDone;
}

/**
 * @brief Start calibrate compass
 * 
 */
void compassCalibrateStart() {
  compassClearCalibration();
  calibrate_c = millis();
  calibrate_t = calibrate_c;
  calibrateChanged = false;
  calibrateDone = false;
  _compass.read();
  calibrationData[0] = _compass.getX();
  calibrationData[1] = _compass.getX();
  calibrationData[2] = _compass.getY();
  calibrationData[3] = _compass.getY();
  calibrationData[4] = _compass.getZ();
  calibrationData[5] = _compass.getZ();
}

/**
 * @brief The loop of  compass calibration
 * 
 */
bool compassCalibrateLoop() {
  if (calibrateDone) {
    return;
  }
  calibrateChanged = false;
  _compass.read();
  int x = _compass.getX();
  int y = _compass.getY();
  int z = _compass.getZ();

  if (x < calibrationData[0]) {
    calibrationData[0] = x;
    calibrateChanged = true;
  } else if (x > calibrationData[1]) {
    calibrationData[1] = x;
    calibrateChanged = true;
  }

  if (y < calibrationData[2]) {
    calibrationData[2] = y;
    calibrateChanged = true;
  } else if (y > calibrationData[3]) {
    calibrationData[3] = y;
    calibrateChanged = true;
  }

  if (z < calibrationData[4]) {
    calibrationData[4] = z;
    calibrateChanged = true;
  } else if (z > calibrationData[5]) {
    calibrationData[5] = z;
    calibrateChanged = true;
  }

  if (calibrateChanged) {
    calibrate_c = millis();
  }
  calibrate_t = millis();
  if (calibrate_t - calibrate_c > CALIBRATION_TIME) {

    compassSaveCalibration();
    calibrateDone = true;
  }
  return calibrateChanged;
}

/**
 * @brief Read the average filtered value of compass angle
 * 
 * @return int16_t average angle
 */
int16_t compassReadAngle() {
  static float filtered = 0.0f;
  static bool initialized = false;
  const float alpha = 0.2f;  // smoothing factor (0–1; lower = smoother)

  _compass.read();
  float raw = (float) compassGetAzimuth();  // 0–360

  if (!initialized) {
    filtered = raw;
    initialized = true;
  } else {
    float diff = raw - filtered;

    // unwrap around 0/360
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    filtered += alpha * diff;

    if (filtered < 0.0f)    filtered += 360.0f;
    if (filtered >= 360.0f) filtered -= 360.0f;
  }

  return (int16_t)filtered;
}


/**
 * @brief Calculate the angle from the values in the x, y direction of the compass sensor
 * 
 * @return int16_t angle
 */
int16_t compassGetAzimuth() {
  // assume _compass.read() was already called this cycle
  int16_t x = _compass.getX();
  int16_t y = _compass.getY();

  // Choose sign/orientation empirically; this is a good starting guess:
  float heading = atan2(-x, y);   // [-pi, pi]

  heading *= 180.0 / PI;          // to degrees

  if (heading < 0) {
    heading += 360.0;
  }

  return (int16_t)heading;
}


/**
 * @brief Compass init 
 * 
 */
void compassBegin() {
  _compass.init();
  compassReadCalibration();
  for (uint8_t i = 0; i < AVERAGE_FILTER_SIZE; i++) {
    compassReadAngle();
  }

}
