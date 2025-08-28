#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

const char* ssid = "MeOrYou";
const char* password = "12345678";

WiFiUDP udp;
IPAddress serverIP;
const int serverPort = 12345;
bool udpReady = false;

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi with 2 second timeout
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 2000) {
    delay(250);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    // Resolve mDNS
    if (MDNS.begin("esp32")) {
      serverIP = MDNS.queryHost("py");
      if (serverIP != INADDR_NONE) {
        udp.begin(8888);
        udpReady = true;
        Serial.print("UDP ready, target: ");
        Serial.println(serverIP);
      } else {
        Serial.println("mDNS resolve failed");
      }
    }
  } else {
    Serial.println("\nWiFi connection failed");
  }
  
  Serial.println("Setup complete, UDP status: " + String(udpReady ? "OK" : "FAILED"));
}

void log(String message) {
  Serial.println("LOG: " + message);
  
  if (udpReady) {
    udp.beginPacket(serverIP, serverPort);
    udp.print(message);
    udp.endPacket();
  }
}

void loop() {
  // Your main code here - non-blocking
  log("System running at " + String(millis()));
  
  // Other non-blocking code
  delay(5000);
}