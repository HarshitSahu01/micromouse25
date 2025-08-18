#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

// ===== Motor Driver Pins =====
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

// ===== Encoder Pins =====
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

// ===== Encoder Variables =====
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ===== PID Variables =====
float targetTicksPerSec = 400;
float Kp_speed = 1.678;
float Ki_speed = -0.020;
float Kd_speed = 0.146;
float Kp_balance = 0.5;

float errorL_prev = 0, errorR_prev = 0;
float integralL = 0, integralR = 0;
int pwmL = 0, pwmR = 0;

unsigned long lastPID = 0;
const unsigned long pidInterval = 100; // ms

// ===== WiFi =====
const char* ssid = "MeOrYou";
const char* password = "12345678";

WebSocketsServer webSocket(81);

// ===== Encoder ISRs =====
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

// ===== Motor Speed Control =====
void mspeed(int a, int b) {
  // Left Motor
  if (a >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, a);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -a);
  }
  // Right Motor
  if (b >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, b);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -b);
  }
}

// ===== WebSocket Event =====
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      if (doc.containsKey("Kp")) Kp_speed = (float)doc["Kp"];
      if (doc.containsKey("Ki")) Ki_speed = (float)doc["Ki"];
      if (doc.containsKey("Kd")) Kd_speed = (float)doc["Kd"];
      if (doc.containsKey("Kb")) Kp_balance = (float)doc["Kb"];
      if (doc.containsKey("target")) targetTicksPerSec = (float)doc["target"];
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Encoders
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

  // mDNS
  if (MDNS.begin("esp")) { Serial.println("mDNS started at esp.local"); }

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("PID Motor Control with WebSocket Ready.");
}

void loop() {
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastPID >= pidInterval) {
    lastPID = now;

    static long prevL = 0, prevR = 0;

    long currL = leftTicks;
    long currR = rightTicks;

    int deltaL = currL - prevL;
    int deltaR = currR - prevR;

    prevL = currL;
    prevR = currR;

    // Speed in ticks/sec
    float ticksPerSecL = (deltaL * 1000.0) / pidInterval;
    float ticksPerSecR = (deltaR * 1000.0) / pidInterval;

    // PID Left
    float errorL = targetTicksPerSec - ticksPerSecL;
    integralL += errorL * (pidInterval / 1000.0);
    float derivativeL = (errorL - errorL_prev) / (pidInterval / 1000.0);
    errorL_prev = errorL;
    float outputL = Kp_speed * errorL + Ki_speed * integralL + Kd_speed * derivativeL;

    // PID Right
    float errorR = targetTicksPerSec - ticksPerSecR;
    integralR += errorR * (pidInterval / 1000.0);
    float derivativeR = (errorR - errorR_prev) / (pidInterval / 1000.0);
    errorR_prev = errorR;
    float outputR = Kp_speed * errorR + Ki_speed * integralR + Kd_speed * derivativeR;

    // Balance
    float balanceError = (ticksPerSecL - ticksPerSecR);
    float balanceCorrection = Kp_balance * balanceError;
    outputL -= balanceCorrection;
    outputR += balanceCorrection;

    // Update PWM
    pwmL = constrain(pwmL + outputL, -255, 255);
    pwmR = constrain(pwmR + outputR, -255, 255);
    mspeed(pwmL, pwmR);

    // Send debug to dashboard
    StaticJsonDocument<200> dbg;
    dbg["L"] = ticksPerSecL;
    dbg["R"] = ticksPerSecR;
    dbg["pwmL"] = pwmL;
    dbg["pwmR"] = pwmR;
    dbg["Kp"] = Kp_speed;
    dbg["Ki"] = Ki_speed;
    dbg["Kd"] = Kd_speed;
    dbg["Kb"] = Kp_balance;
    dbg["target"] = targetTicksPerSec;
    String msg;
    serializeJson(dbg, msg);
    webSocket.broadcastTXT(msg);

    // Serial debug
    Serial.println(msg);
  }
}
