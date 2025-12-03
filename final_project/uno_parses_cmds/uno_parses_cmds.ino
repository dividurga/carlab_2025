#include "car_control.h"   // your existing file with carMove, carStop, etc.
#include "compass.h"
#include <Servo.h>
String buf = "";

Servo servo;
int servoPin = 9;

void setup() {
  Serial.begin(115200);      // UART from ESP32-CAM (pins 0 and 1)
  carBegin();                // initialize motors, PWM, etc.
  Serial.println("UNO car controller ready");
  delay(1);
  servo.attach(servoPin);
  servo.write(90);
}

int angle = 90;

// --------------------------------------------------------------------
//  Parse one full line and call the corresponding low-level functions
// --------------------------------------------------------------------
void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  

  // ---- split command ----
  int firstSpace = line.indexOf(' ');
  String cmd = (firstSpace == -1) ? line : line.substring(0, firstSpace);
  cmd.toUpperCase();

  // -------------------------------------------------
  // MOVE <angle> <power> <rot> <drift>
  // -------------------------------------------------
  if (cmd == "MOVE") {
    int args[4] = {0, 0, 0, 0};
    int idx = 0;
    int start = firstSpace + 1;
    while (start < line.length() && idx < 4) {
      int next = line.indexOf(' ', start);
      if (next == -1) next = line.length();
      args[idx++] = line.substring(start, next).toInt();
      start = next + 1;
    }

    int angle = args[0];
    int power = args[1];
    int rot   = args[2];
    bool drift = (args[3] != 0);

    Serial.print("MOVE: angle="); Serial.print(angle);
    Serial.print("  power="); Serial.print(power);
    Serial.print("  rot="); Serial.print(rot);
    Serial.print("  drift="); Serial.println(drift);

    carMove(angle, power, rot, drift);
  }

  // -------------------------------------------------
  // STOP
  // -------------------------------------------------
  else if (cmd == "STOP") {
    Serial.println("STOP");
    carStop();
  }
  // -------------------------------------------------
  // MOTORS <p1> <p2> <p3> <p4>
  // -------------------------------------------------
  else if (cmd == "MOTORS") {
    int p[4] = {0, 0, 0, 0};
    int start = firstSpace + 1;
    for (int i = 0; i < 4 && start < line.length(); i++) {
      int next = line.indexOf(' ', start);
      if (next == -1) next = line.length();
      p[i] = line.substring(start, next).toInt();
      start = next + 1;
    }

    Serial.print("MOTORS: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(p[i]);
      if (i < 3) Serial.print("  ");
    }
    Serial.println();

    carSetMotors(p[0], p[1], p[2], p[3]);
  }
  else if (cmd == "RST") {
    Serial.println("RESET_HEADING");
    carResetHeading();
    
  }
  else if (cmd == "CALIBRATE") {
    Serial.println("Starting compass calibration...");
    compassCalibrate();
}
else if (cmd == "READ") {
    Serial.println(compassReadAngle());
}
  // move from 90 to 200 degrees
  else if (cmd == "D"){
  for(int angle = 90; angle < 180; angle++)  
  {                                  
    servo.write(angle);               
    delayMicroseconds(500);                   
  } 
  }
  else if (cmd == "UP"){
  // move back from 200 to 90 degrees
  for(int angle = 180; angle > 90; angle--)    
  {                                
    servo.write(angle);           
    delayMicroseconds(500);       
  } 
  }
  // -------------------------------------------------
  // Unknown command
  // -------------------------------------------------
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

// --------------------------------------------------------------------
//  Read serial characters from ESP32-CAM and accumulate full lines
// --------------------------------------------------------------------
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;      // ignore CR
    if (c == '\n') {              // end of command
      processCommand(buf);
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 200) buf = "";   // safety clear
    }
  }  
  
  
}
void compassCalibrate() {
  Serial.println("Starting compass calibration...");

  const uint32_t CALIB_TIMEOUT_MS = 90000;   // 90 seconds
  uint32_t startTime = millis();

  // Start spinning CCW for calibration
  carMove(0, 0, 100);

  // Start collecting calibration samples
  compassCalibrateStart();

  while (!compassCalibrateDone()) {

    // --- TIMEOUT CHECK ---
    if (millis() - startTime > CALIB_TIMEOUT_MS) {
      Serial.println("Calibration timed out.");
      break;
    }

    // --- Perform one calibration step ---
    bool changed = compassCalibrateLoop();

    if (changed) {
      Serial.println("calibrated some");
    }
  }

  // Stop rotating
  carStop();
  Serial.println("Compass calibration done (or timed out).");

  delay(1000);
}