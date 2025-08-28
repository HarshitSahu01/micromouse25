#include "wifilogger.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

static WiFiUDP udp;
static IPAddress serverIP;
static int serverPort;
static bool udpReady = false;

void initWiFiLogger(const char* ssid, const char* password, const char* targetHost, int port) {
  serverPort = port;
  
  Serial.print("WiFiLogger: Connecting to ");
  Serial.print(ssid);
  
  // Connect to WiFi with 2 second timeout
  WiFi.begin(ssid, password);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 2000) {
    delay(250);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.print("WiFiLogger: IP ");
    Serial.println(WiFi.localIP());
    
    // Resolve mDNS
    if (MDNS.begin("esp32")) {
      serverIP = MDNS.queryHost(targetHost);
      if (serverIP != INADDR_NONE) {
        udp.begin(8888);
        udpReady = true;
        Serial.print("WiFiLogger: UDP ready -> ");
        Serial.print(serverIP);
        Serial.print(":");
        Serial.println(serverPort);
      } else {
        Serial.print("WiFiLogger: mDNS resolve failed for ");
        Serial.println(targetHost);
      }
    } else {
      Serial.println("WiFiLogger: mDNS init failed");
    }
  } else {
    Serial.println(" FAILED");
  }
  
  Serial.print("WiFiLogger: Status ");
  Serial.println(udpReady ? "READY" : "FAILED");
}

void log(String message) {
  Serial.println("LOG: " + message);
  
  if (udpReady) {
    udp.beginPacket(serverIP, serverPort);
    udp.print(message);
    udp.endPacket();
  }
}

bool isLoggerReady() {
  return udpReady;
}