// esp32cam_uartbridge_oneway.ino
#include <WiFi.h>

const char* AP_SSID = "ZeusBridge_AP";
const char* AP_PASS = "zeuscar123";

const uint16_t TCP_PORT = 8888;
WiFiServer server(TCP_PORT);
WiFiClient client;

void setup() {
  Serial.begin(115200);   // UART0 -> Uno RX (GPIO1 TX0)
  delay(200);
  Serial.println("\nESP32-CAM one-way UART bridge starting...");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP started: %s  IP=%s\n", AP_SSID, ip.toString().c_str());

  server.begin();
  Serial.printf("TCP server on port %u ready.\n", TCP_PORT);
}

String buf = "";

void loop() {
  // accept one client
  if (!client || !client.connected()) {
    if (server.hasClient()) {
      client = server.available();
      client.setNoDelay(true);
      Serial.println("TCP client connected.");
    }
  }

  // read from TCP and forward to Uno
  if (client && client.connected() && client.available()) {
    while (client.available()) {
      char c = client.read();
      if (c == '\r') continue;
      buf += c;
      if (c == '\n') {
        buf.trim();
        if (buf.length() > 0) {
          Serial.print("[BRIDGE -> UNO] ");
          Serial.println(buf);
          // send raw command to Uno
          Serial.print(buf);
          Serial.print('\n');
        }
        buf = "";
      }
    }
  }

  delay(1);
}
