#include "car_control.h"   // your existing file with carMove, carStop, etc.

String buf = "";

void setup() {
  Serial.begin(115200);      // UART from ESP32-CAM (pins 0 and 1)
  carBegin();                // initialize motors, PWM, etc.
  Serial.println("UNO car controller ready");
}

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
  // RESET_HEADING
  // -------------------------------------------------
  else if (cmd == "RESET_HEADING") {
    Serial.println("RESET_HEADING");
    carResetHeading();
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
