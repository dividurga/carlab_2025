#include <Arduino.h>
#include "rgb.h"
#include "car_control.h"
#include "ai_camera.h"

// --------------------------- Configuration ---------------------------
#define WIFI_MODE WIFI_MODE_AP
#define SSID "Zeus_Car"
#define PASSWORD "12345678"
#define NAME "Zeus_Car"
#define TYPE "Zeus_Car"
#define PORT "8765"

// --------------------------- Globals ---------------------------
AiCamera aiCam(NAME, TYPE);
int16_t remoteAngle = 0;
int16_t remotePower = 0;
int16_t remoteHeading = 0;
int16_t remoteHeadingR = 0;
bool remoteDriftEnable = false;

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(115200);
  rgbBegin();
  rgbWrite(0xFF2500); // orange = booting
  carBegin();

  aiCam.begin(SSID, PASSWORD, WIFI_MODE, PORT);
  aiCam.setOnReceived(onReceive);

  delay(500);
  Serial.println(F("UNO ready, connected to ESP32-CAM"));
  rgbWrite(0x00FF00); // green = ready
}

// --------------------------- Main Loop ---------------------------
void loop() {
  aiCam.loop();
  carMoveFieldCentric(remoteAngle, remotePower, remoteHeading, remoteDriftEnable);
}

// --------------------------- OnReceive ---------------------------
void onReceive(char* recvBuf, char* sendBuf) {
  // Joystick [K]
  uint16_t angle = aiCam.getJoystick(REGION_K, JOYSTICK_ANGLE);
  uint8_t power  = aiCam.getJoystick(REGION_K, JOYSTICK_RADIUS);
  power = map(power, 0, 100, 0, CAR_DEFAULT_POWER);

  if (remoteAngle != angle) remoteAngle = angle;
  if (remotePower != power) remotePower = power;

  // Drift [J]
  bool drift = aiCam.getSwitch(REGION_J);
  if (remoteDriftEnable != drift) {
    remoteDriftEnable = drift;
  }

  // Move Head [Q]
  int moveHeadingA  = aiCam.getJoystick(REGION_Q, JOYSTICK_ANGLE);
  int16_t moveHeadingR = aiCam.getJoystick(REGION_Q, JOYSTICK_RADIUS);

  if ((remoteHeading != moveHeadingA) || (remoteHeadingR != moveHeadingR)) {
    remoteHeading  = moveHeadingA;
    remoteHeadingR = moveHeadingR;
    if (remoteDriftEnable && moveHeadingR == 0) {
      carResetHeading();
      remoteHeading = 0;
    }
  }
}
