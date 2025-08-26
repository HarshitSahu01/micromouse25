#include <SparkFun_TB6612.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>


// ---------------- Motor driver pins ----------------
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

const int offsetA = 1;
const int offsetB = 1;
Motor motorLeft(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight(BIN1, BIN2, PWMB, offsetB, STBY);

// ---------------- ToF sensor setup ----------------
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = { 13, 25, 27 }; // L, R, F
VL53L1X sensors[sensorCount];

// ---------------- PID variables ----------------
float wallKp = 1.05;   // start small
float wallKi = 0.001;  // start near zero
float wallKd = 0.2;   // start small

float wallError = 0;
float wallPrev  = 0;
float wallInt   = 0;
float wallPIDValue = 0;

int baseSpeed = 240; // start slow for tuning

// ---------------- Distances ----------------
int distLeft, distRight, distFront;

/* Websockets */

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
      if (jd.containsKey("Kp")) { wallKp = jd["Kp"].as<float>(); debugMsg += "Kp = " + String(wallKp) + "\n"; Serial.println("Updated Kp: " + String(wallKp)); }
      if (jd.containsKey("Ki")) { wallKi = jd["Ki"].as<float>(); debugMsg += "Ki = " + String(wallKi) + "\n"; Serial.println("Updated Ki: " + String(wallKi)); }
      if (jd.containsKey("Kd")) { wallKd = jd["Kd"].as<float>(); debugMsg += "Kd = " + String(wallKd) + "\n"; Serial.println("Updated Kd: " + String(wallKd)); }
    }
  }
}


void updateSensors() {
  distLeft  = sensors[0].read();
  distRight = sensors[1].read();
  distFront = sensors[2].read();
}

int targetLeftDist = 80; // mm
void runRightWallPID(int dR) {
  int error =  targetLeftDist - dR;         // error = actual - desired distance
  wallInt += error;                        // integral term
  wallInt = constrain(wallInt, -1000, 1000); // anti-windup
  float deriv = error - wallPrev;          // derivative term
  wallPrev = error;

  float pidVal = wallKp * error + wallKi * wallInt + wallKd * deriv;

  int leftSpeed  = constrain(baseSpeed - pidVal, 0, 255);  // slow down if too close
  int rightSpeed = constrain(baseSpeed + pidVal, 0, 255);  // speed up opposite wheel

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  // Debug
  Serial.print("R:"); Serial.print(dR);
  Serial.print("\tErr:"); Serial.print(error);
  Serial.print("\tPID:"); Serial.print(pidVal);
  Serial.print("\tLspd:"); Serial.print(leftSpeed);
  Serial.print("\tRspd:"); Serial.println(rightSpeed);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 32);   // ESP32 custom I2C pins
  Wire.setClock(400000);

  // Reset all ToF sensors
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Bring up sensors one by one
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect sensor "); Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i);
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous(20);
  }

  Serial.println("Sensors initialized. Running Both-Wall PID only.");

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
  if (running) {
    updateSensors();
  // runWallPID(distLeft, distRight);
  // runLeftWallPID(distLeft);
  runRightWallPID(distRight);
  delay(2); // 50 Hz loop
  }
  else {
    motorLeft.drive(0);
    motorRight.drive(0);
  }
}