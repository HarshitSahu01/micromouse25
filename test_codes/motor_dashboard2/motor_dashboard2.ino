// File: motor_dashboard2.ino

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <VL53L1X.h>  // Pololu VL53L1X library

// === PIN DEFINITIONS ===
// Motors
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

// Encoders
#define LEFT_ENC_A  2
#define LEFT_ENC_B  15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

// VL53L1X XSHUT pins
#define XSHUT_L 13
#define XSHUT_F 27
#define XSHUT_R 25

// === GLOBALS ===
volatile long leftTicks = 0, rightTicks = 0;
int speedA = 0, speedB = 0;
int timingBudget = 33;  // ms

bool initL = false, initF = false, initR = false;
bool clientConnected = false;

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");
String           debugMsg;

// Pololu VL53L1X instances
VL53L1X sensorL;
VL53L1X sensorF;
VL53L1X sensorR;

// === ISRs ===
void IRAM_ATTR onLeftEnc() {
  bool a = digitalRead(LEFT_ENC_A), b = digitalRead(LEFT_ENC_B);
  leftTicks += (a == b) ? 1 : -1;
}
void IRAM_ATTR onRightEnc() {
  bool a = digitalRead(RIGHT_ENC_A), b = digitalRead(RIGHT_ENC_B);
  rightTicks += (a == b) ? 1 : -1;
}

// === MOTOR SETUP & CONTROL ===
void setupMotors() {
  pinMode(PWMA, OUTPUT);  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);  digitalWrite(STBY, HIGH);
}
void mspeed(int a, int b) {
  // Left
  if (a >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, a);
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -a);
  }
  // Right
  if (b >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, b);
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -b);
  }
}

// === SENSOR INITIALIZATION ===
void initSensors() {
  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);

  // Hold all off
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(10);

  auto cfg = [&](VL53L1X &lox, uint8_t pin, bool &flag, const char *name) {
    digitalWrite(pin, HIGH);
    delay(10);
    if (!lox.init()) {
      debugMsg += String(name) + " init FAILED\n";
      flag = false;
    } else {
      lox.setDistanceMode(VL53L1X::Medium);
      lox.setMeasurementTimingBudget(timingBudget * 1000);    // <— Pololu API
      lox.startContinuous(timingBudget);                       // period in ms
      debugMsg += String(name) + " init OK\n";
      flag = true;
    }
  };

  cfg(sensorL, XSHUT_L, initL, "Left Sensor");
  cfg(sensorF, XSHUT_F, initF, "Front Sensor");
  cfg(sensorR, XSHUT_R, initR, "Right Sensor");
}

// === WEBSOCKET HANDLER ===
void notifyClients() {
  StaticJsonDocument<512> doc;
  doc["conn"]  = clientConnected;
  doc["initL"] = initL;  doc["initF"] = initF;  doc["initR"] = initR;
  doc["lt"]    = leftTicks;
  doc["rt"]    = rightTicks;

  // Read continuous ranges
  if (initL) {
    if (!sensorL.read(false)) doc["sl"] = sensorL.ranging_data.range_mm;
    else                       doc["sl"] = -1;
  } else doc["sl"] = -1;

  if (initF) {
    if (!sensorF.read(false)) doc["sf"] = sensorF.ranging_data.range_mm;
    else                       doc["sf"] = -1;
  } else doc["sf"] = -1;

  if (initR) {
    if (!sensorR.read(false)) doc["sr"] = sensorR.ranging_data.range_mm;
    else                       doc["sr"] = -1;
  } else doc["sr"] = -1;

  doc["debug"] = debugMsg;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
  debugMsg = "";
}

// Correct AwsEventHandler signature
void onWsEvent(AsyncWebSocket *server,
               AsyncWebSocketClient *client,
               AwsEventType type,
               void *arg,
               uint8_t *data,
               size_t len) {
  if (type == WS_EVT_CONNECT) {
    clientConnected = true;
    debugMsg += "Client Connected\n";
  } else if (type == WS_EVT_DISCONNECT) {
    clientConnected = false;
    debugMsg += "Client Disconnected\n";
  } else if (type == WS_EVT_DATA) {
    String msg; msg.reserve(len + 1);
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    StaticJsonDocument<256> jd;
    if (!deserializeJson(jd, msg)) {
      if (jd.containsKey("sa")) {
        int inSa = jd["sa"].as<int>();
        speedA = constrain(inSa, -255, 255);
        debugMsg += "Left speed → " + String(speedA) + "\n";
      }
      if (jd.containsKey("sb")) {
        int inSb = jd["sb"].as<int>();
        speedB = constrain(inSb, -255, 255);
        debugMsg += "Right speed → " + String(speedB) + "\n";
      }
      if (jd.containsKey("stop")) {
        speedA = speedB = 0;
        debugMsg += "Motors stopped\n";
      }
      if (jd.containsKey("tb")) {
        timingBudget = jd["tb"].as<int>();
        debugMsg += "TimingBudget → " + String(timingBudget) + " ms\n";
        if (initL) sensorL.setMeasurementTimingBudget(timingBudget * 1000);
        if (initF) sensorF.setMeasurementTimingBudget(timingBudget * 1000);
        if (initR) sensorR.setMeasurementTimingBudget(timingBudget * 1000);
      }
      mspeed(speedA, speedB);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setupMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  onLeftEnc,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), onRightEnc, RISING);

  WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  if (!MDNS.begin("esp32")) Serial.println("mDNS init failed");

  initSensors();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // Push every 2 s
  xTaskCreatePinnedToCore([](void*) {
    while (1) {
      notifyClients();
      delay(2000);
    }
  }, "pushTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // All handled via interrupts & tasks
}
