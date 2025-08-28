/*
  esp32_pid_trial.ino
  - WebSocket server at /ws
  - Accepts JSON commands: {"Kp":..,"Ki":..,"Kd":..}, {"start":50}, {"stop":true}
  - When a start arrives, run a single forward trial to targetDistance (cm)
  - Streams periodic "error" messages and a final "result" message
*/

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

// ---------------- Encoder pins (from your baseline) ----
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Encoder ISR
void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  leftTicks += (a == b) ? 1 : -1;
}
void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  rightTicks += (a == b) ? 1 : -1;
}

// ---------------- PID / movement ----------------
float wallKp = 1.05;
float wallKi = 0.001;
float wallKd = 0.2;

float wallError = 0, wallPrev = 0, wallInt = 0, wallPIDValue = 0;
int baseSpeed = 150; // tuning start
int timingBudget = 20; // ms loop delay

float cmToEncoderTicks = 11.4; // your mapping (~ticks per cm)

// simple mspeed wrapper for Motor class (-255..255)
void mspeed(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);
  motorLeft.drive(a);
  motorRight.drive(b);
}

// very small runWallPID using encoder balance (fallback)
void runEncoderPID() {
  wallError = (float)leftTicks - (float)rightTicks;
  wallInt += wallError;
  wallInt = constrain(wallInt, -10000, 10000);
  float deriv = wallError - wallPrev;
  wallPrev = wallError;
  wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * deriv;
  int leftSpeed  = constrain(baseSpeed - (int)wallPIDValue, -255, 255);
  int rightSpeed = constrain(baseSpeed + (int)wallPIDValue, -255, 255);
  mspeed(leftSpeed, rightSpeed);
}

// ---------------- Websocket server ----------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool clientConnected = false;
bool running = false;
float targetDistance = 0.0; // cm
bool runActive = false;
String debugMsg = "";

// helper: send a JSON object to all ws clients
void wsSendJson(JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// telemetry / heartbeat
void notifyClients() {
  StaticJsonDocument<256> doc;
  doc["conn"] = clientConnected;
  doc["running"] = running;
  doc["distance"] = targetDistance;
  doc["Kp"] = wallKp;
  doc["Ki"] = wallKi;
  doc["Kd"] = wallKd;
  doc["debug"] = debugMsg;
  wsSendJson(doc);
  debugMsg = "";
}

// handle incoming websocket data
void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    clientConnected = true;
    Serial.println("Client Connected");
    return;
  } 
  if (type == WS_EVT_DISCONNECT) {
    clientConnected = false;
    Serial.println("Client Disconnected");
    return;
  }
  if (type == WS_EVT_DATA) {
    // assemble string
    String msg; msg.reserve(len + 1);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    StaticJsonDocument<256> jd;
    DeserializationError err = deserializeJson(jd, msg);
    if (err) {
      Serial.println("Bad JSON from client");
      return;
    }

    if (jd.containsKey("start")) {
      targetDistance = jd["start"].as<float>();
      running = true;         // request run; actual run performed in loop()
      debugMsg += "Start command: " + String(targetDistance) + " cm\n";
      Serial.println("START received, dist " + String(targetDistance));
    }
    if (jd.containsKey("stop")) {
      running = false;
      debugMsg += "Stop command received\n";
      Serial.println("STOP received");
    }
    if (jd.containsKey("Kp")) {
      wallKp = jd["Kp"].as<float>();
      debugMsg += "Kp=" + String(wallKp) + "\n";
      Serial.println("Kp updated: " + String(wallKp));
    }
    if (jd.containsKey("Ki")) {
      wallKi = jd["Ki"].as<float>();
      debugMsg += "Ki=" + String(wallKi) + "\n";
      Serial.println("Ki updated: " + String(wallKi));
    }
    if (jd.containsKey("Kd")) {
      wallKd = jd["Kd"].as<float>();
      debugMsg += "Kd=" + String(wallKd) + "\n";
      Serial.println("Kd updated: " + String(wallKd));
    }
  }
}

// ---------------- Trial runner ----------------
// compute final metrics after run, and stream periodic errors
void runTrial(float dist_cm) {
  runActive = true;
  // reset
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  long targetTicks = (long)round(dist_cm * cmToEncoderTicks);
  unsigned long startTime = millis();
  unsigned long lastSend = 0;
  long maxAvgTicks = 0;
  const unsigned long safetyTimeout = 15000; // 15 s max per trial
  bool unstable = false;

  // record errors for RMS calculation
  const int maxSamples = 5000;
  // dynamic vector-like structures on ESP32 -> use simple arrays up to a cap
  static float errSquares[2000];
  int errCount = 0;

  // Reset integrators
  wallInt = 0; wallPrev = 0;

  while (running) {
    // check safety timeout
    if (millis() - startTime > safetyTimeout) {
      unstable = true;
      Serial.println("Trial timed out -> marking unstable");
      break;
    }

    // update sensors & encoders
    // note: sensors may block if not ready — keep timingBudget small
    for (uint8_t i=0;i<sensorCount;i++) {
      // read continuously started sensors
      // no per-sensor read call is wrapped here; VL53L1X.read() is used in other sketches
      // we will not rely heavily on ToF for metrics here; using encoders primarily
    }

    // control: use encoder PID to maintain heading and drive forward
    runEncoderPID();

    // compute progress / error
    noInterrupts();
    long L = leftTicks;
    long R = rightTicks;
    interrupts();
    long avgTicks = (abs(L) + abs(R)) / 2;
    long diffTicks = L - R; // heading error in ticks
    if (avgTicks > maxAvgTicks) maxAvgTicks = avgTicks;

    // save squared error (ticks)
    if (errCount < 2000) {
      float mmError = (float)diffTicks / (cmToEncoderTicks / 10.0f); // ticks -> mm (ticks per mm = ticks/cm / 10)
      errSquares[errCount++] = mmError * mmError;
    }

    // send periodic error messages every 200 ms
    if (millis() - lastSend >= 200) {
      lastSend = millis();
      StaticJsonDocument<128> d;
      d["type"] = "error";
      d["time_ms"] = (int)(millis() - startTime);
      d["error_ticks"] = diffTicks;
      d["avg_ticks"] = avgTicks;
      wsSendJson(d);
    }

    // check if target reached
    if (avgTicks >= targetTicks) {
      break;
    }

    delay( (unsigned long) timingBudget );
  } // while running

  // stop motors
  mspeed(0, 0);

  unsigned long endTime = millis();
  float runtime_s = (endTime - startTime) / 1000.0f;

  // compute metrics
  float rms_mm = 0.0f;
  if (errCount > 0) {
    float meanSq = 0.0f;
    for (int i=0;i<errCount;i++) meanSq += errSquares[i];
    meanSq /= errCount;
    rms_mm = sqrt(meanSq);
  }
  float overshoot_frac = 0.0f;
  if (maxAvgTicks > targetTicks) {
    overshoot_frac = (float)(maxAvgTicks - targetTicks) / (float)targetTicks;
  }

  // settling time heuristic: time until avgTicks reached within +/- tolerance
  // simple fallback: set settling time to runtime
  float settling_s = runtime_s;

  // mark unstable if overshoot too big
  if (overshoot_frac > 0.25f) unstable = true;

  // final result message
  StaticJsonDocument<256> result;
  result["type"] = "result";
  result["rms_error_mm"] = rms_mm;
  result["overshoot_frac"] = overshoot_frac;
  result["settling_time_s"] = settling_s;
  result["unstable"] = unstable;
  result["avg_target_ticks"] = (long)targetTicks;
  result["max_ticks"] = maxAvgTicks;
  wsSendJson(result);

  // clear running flag
  running = false;
  runActive = false;
  Serial.println("Trial finished, result sent.");
}

// ---------------- Setup & loop ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C
  Wire.begin(33, 32);
  Wire.setClock(400000);

  // Init sensors
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(100);
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
  Serial.println("Sensors initialized");

  // encoders
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  Serial.println("Encoders initialized");

  // WiFi
  WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  if (!MDNS.begin("esp32")) Serial.println("mDNS init failed");

  // Websocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // periodic notify on separate task
  xTaskCreatePinnedToCore([](void*) {
    while (true) {
      notifyClients();
      delay(5000);
    }
  }, "pushTask", 4096, NULL, 1, NULL, 1);

  Serial.println("Setup complete, waiting for commands...");
}

void loop() {
  // If a start was requested and we're not already running a trial, run it
  if (running && !runActive) {
    runTrial(targetDistance > 0 ? targetDistance : 50.0); // default 50 cm if not provided
  }

  // Otherwise idle; websocket handlers and pushTask handle comms
  delay(50);
}
