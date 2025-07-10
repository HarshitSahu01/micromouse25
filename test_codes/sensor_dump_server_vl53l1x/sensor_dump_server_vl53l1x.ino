#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESPmDNS.h>

// === WiFi credentials ===
const char* ssid = "MeOrYou";
const char* password = "12345678";

// === Web server and sensor ===
AsyncWebServer server(80);
VL53L1X sensor;

// === Configuration ===
uint16_t timingBudget = 20000; // µs

#define MAX_READINGS 300
uint16_t readings[MAX_READINGS];
int readingCount = 0;
int readingIndex = 0;
bool collecting = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 for ESP32

  // === Connect WiFi ===
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // === mDNS ===
  if (!MDNS.begin("esp")) {
    Serial.println("mDNS failed");
  } else {
    Serial.println("Available at: http://esp.local");
  }

  // === Sensor setup ===
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("VL53L1X not found");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Short); // Short-range for accuracy
  sensor.setMeasurementTimingBudget(timingBudget);
  sensor.startContinuous(timingBudget / 1000);

  // === CORS header ===
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  // === Live endpoint ===
  server.on("/live", HTTP_GET, [](AsyncWebServerRequest *request){

    uint16_t d = sensor.read();
    String json = "{\"distance\":" + String(d) + "}";
    request->send(200, "application/json", json);
  });

server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
  int readingCount = request->getParam("count")->value().toInt();
  if (readingCount <= 0 || readingCount > MAX_READINGS) {
    request->send(400, "application/json", "{\"error\":\"Invalid or missing reading count.\"}");
    return;
  }

  // Start measurement now
  for (int i = 0; i < readingCount; i++) {
    readings[i] = sensor.read();
    delay(timingBudget / 1000);  // Wait based on timing budget
  }
  // ready = true;

  // Build response
  String json = "[";
  for (int i = 0; i < readingCount; i++) {
    json += String(readings[i]);
    if (i < readingCount - 1) json += ",";
  }
  json += "]";

  AsyncResponseStream *response = request->beginResponseStream("application/json");
  response->print(json);
  request->send(response);

  // ready = false; // Reset
});

  // === Timing budget update ===
  server.on("/timingBudget", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("val")) {
      request->send(400, "text/plain", "Missing val param");
      return;
    }

    int val = request->getParam("val")->value().toInt();

    timingBudget = val;
    sensor.setMeasurementTimingBudget(timingBudget);
    sensor.startContinuous(timingBudget / 1000);
    request->send(200, "text/plain", "Timing budget updated to " + String(val));
  });

  server.begin();
}

void loop() {
  // === Pull mode reading loop ===
}
