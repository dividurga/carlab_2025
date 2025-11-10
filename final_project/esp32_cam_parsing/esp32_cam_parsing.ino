// uno_receive_only.ino
// Just reads commands from ESP32-CAM and optionally prints to USB for debugging.

void setup() {
  Serial.begin(115200);   // Hardware Serial (pins 0/1) connected to ESP
  Serial.println("UNO ready to receive from ESP...");
}

String buf = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf.trim();
      if (buf.length() > 0) {
        // Replace this with your car control function
        // e.g., parse MOVE / MOTORS commands here
        Serial.print("Received command: ");
        Serial.println(buf);
      }
      buf = "";
    } else {
      buf += c;
    }
  }
}
