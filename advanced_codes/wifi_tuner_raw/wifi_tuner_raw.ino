#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// === GLOBALS ===
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool clientConnected = false;
bool running = false;
float targetDistance = 0.0;

float Kp = 1.0, Ki = 0.0, Kd = 0.0;
String debugMsg = "";

// === WEBSOCKET HANDLER ===
void notifyClients() {
  StaticJsonDocument<256> doc;
  doc["conn"] = clientConnected;
  doc["running"] = running;
  doc["distance"] = targetDistance;
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["debug"] = debugMsg;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);

  // Serial debug
  if(debugMsg.length() > 0){
    Serial.print(debugMsg);
  }
  debugMsg = "";
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    clientConnected = true;
    debugMsg += "Client Connected\n";
    Serial.println("Client Connected");
  } else if (type == WS_EVT_DISCONNECT) {
    clientConnected = false;
    debugMsg += "Client Disconnected\n";
    Serial.println("Client Disconnected");
  } else if (type == WS_EVT_DATA) {
    String msg; msg.reserve(len+1);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    StaticJsonDocument<256> jd;
    if (!deserializeJson(jd, msg)) {
      if (jd.containsKey("start")) {
        targetDistance = jd["start"].as<float>();
        running = true;
        debugMsg += "Started → targetDistance: " + String(targetDistance) + "\n";
        Serial.println("Received START command, distance: " + String(targetDistance));
      }
      if (jd.containsKey("stop")) {
        running = false;
        debugMsg += "Stopped\n";
        Serial.println("Received STOP command");
      }
      if (jd.containsKey("Kp")) { Kp = jd["Kp"].as<float>(); debugMsg += "Kp = " + String(Kp) + "\n"; Serial.println("Updated Kp: " + String(Kp)); }
      if (jd.containsKey("Ki")) { Ki = jd["Ki"].as<float>(); debugMsg += "Ki = " + String(Ki) + "\n"; Serial.println("Updated Ki: " + String(Ki)); }
      if (jd.containsKey("Kd")) { Kd = jd["Kd"].as<float>(); debugMsg += "Kd = " + String(Kd) + "\n"; Serial.println("Updated Kd: " + String(Kd)); }
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  if (!MDNS.begin("esp32")) Serial.println("mDNS init failed");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // Push updates every 2s
  xTaskCreatePinnedToCore([](void*) {
    while (1) {
      notifyClients();
      delay(5000);
    }
  }, "pushTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Nothing here; handled via WebSocket & task
}
