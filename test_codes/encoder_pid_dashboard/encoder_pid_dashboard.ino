#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Motor pins
#define AIN1 16
#define AIN2 17
#define PWMA 4
#define BIN1 19
#define BIN2 18
#define PWMB 21
#define STBY 5

// Encoder pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

int baseSpeed = 100, maxSpeed = 255;
float Kp = 1.0, Ki = 0.0, Kd = 0.0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool botRunning = false;
unsigned long lastControlTime = 0, lastSendTime = 0;
float integral = 0, lastError = 0;

void IRAM_ATTR onLeftA() {
  if (digitalRead(LEFT_ENC_B)) leftTicks--; else leftTicks++;
}
void IRAM_ATTR onRightA() {
  if (digitalRead(RIGHT_ENC_B)) rightTicks--; else rightTicks++;
}

void setMotor(int pwmA, int pwmB) {
  digitalWrite(STBY, HIGH);
  // Left
  digitalWrite(AIN1, pwmA >= 0);
  digitalWrite(AIN2, pwmA < 0);
  analogWrite(PWMA, abs(pwmA));
  // Right
  digitalWrite(BIN1, pwmB >= 0);
  digitalWrite(BIN2, pwmB < 0);
  analogWrite(PWMB, abs(pwmB));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) return;

  if (doc.containsKey("run")) {
    botRunning = doc["run"];
  }
  if (doc.containsKey("base")) baseSpeed = doc["base"];
  if (doc.containsKey("max")) maxSpeed = doc["max"];
  if (doc.containsKey("kp")) Kp = doc["kp"];
  if (doc.containsKey("ki")) Ki = doc["ki"];
  if (doc.containsKey("kd")) Kd = doc["kd"];
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text("Connected to ESP32.");
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), onLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), onRightA, RISING);

  WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  Serial.println("WiFi connected");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  unsigned long now = millis();

  if (botRunning && now - lastControlTime >= 100) {
    lastControlTime = now;

    long error = leftTicks - rightTicks;
    integral += error;
    float derivative = error - lastError;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    int pwmL = constrain(baseSpeed - correction, -maxSpeed, maxSpeed);
    int pwmR = constrain(baseSpeed + correction, -maxSpeed, maxSpeed);
    setMotor(pwmL, pwmR);
  }

  if (!botRunning) {
    setMotor(0, 0);
    integral = lastError = 0;
  }

  if (now - lastSendTime >= 3000) {
    lastSendTime = now;
    String msg = "Left ticks: " + String(leftTicks) + ", Right ticks: " + String(rightTicks);
    ws.textAll(msg);
  }
}
