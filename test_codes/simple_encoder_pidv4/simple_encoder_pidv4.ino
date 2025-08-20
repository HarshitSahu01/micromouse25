#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

// ===== WiFi Config =====
const char* ssid = "MeOrYou";
const char* password = "12345678";

WebSocketsServer webSocket(81);

// ===== Motor + Encoder Pins =====
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ===== PID Variables =====
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float baseSpeed = 120;
float integral = 0, prevError = 0;

// ===== ISR for Encoders =====
void IRAM_ATTR leftISR() {
  int b = digitalRead(LEFT_ENC_B);
  leftTicks += (b == HIGH) ? 1 : -1;
}
void IRAM_ATTR rightISR() {
  int b = digitalRead(RIGHT_ENC_B);
  rightTicks += (b == HIGH) ? 1 : -1;
}

// ===== Motor Control =====
void mspeed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); left = -left; }

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); right = -right; }

  analogWrite(PWMA, left);
  analogWrite(PWMB, right);
}

void stopMotors() {
  mspeed(0, 0);
}

// ===== Run Trial (simulate error accumulation) =====
float runTrial() {
  leftTicks = rightTicks = 0;
  integral = 0; prevError = 0;

  unsigned long start = millis();
  float totalError = 0;

  while (millis() - start < 3000) { // run for 3 sec
    long error = (long)leftTicks - (long)rightTicks;

    integral += error;
    float derivative = error - prevError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;

    int L = (int)(baseSpeed - correction);
    int R = (int)(baseSpeed + correction);
    mspeed(L, R);

    totalError += abs(error);
    delay(20);
  }

  stopMotors();
  return totalError;
}

// ===== Handle WebSocket =====
void onMessage(uint8_t num, uint8_t *payload, size_t length) {
  payload[length] = '\0';
  DynamicJsonDocument doc(256);
  deserializeJson(doc, payload);

  if (doc.containsKey("Kp")) {
    Kp = (float)doc["Kp"];
    Ki = (float)doc["Ki"];
    Kd = (float)doc["Kd"];
    baseSpeed = (float)doc["base"];

    float err = runTrial();

    DynamicJsonDocument res(128);
    res["error"] = (float)err;
    String out;
    serializeJson(res, out);
    webSocket.sendTXT(num, out);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, RISING);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println(WiFi.localIP());

  if (!MDNS.begin("esp32")) {
    Serial.println("Error starting mDNS");
  }

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
    if (type == WStype_TEXT) onMessage(num, payload, len);
  });
}

void loop() {
  webSocket.loop();
}
