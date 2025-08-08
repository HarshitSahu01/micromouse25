#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

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

// Sensor mock data
int sensorLeft = 101;
int sensorFront = 92;
int sensorRight = 97;

bool sensorInit[3] = {false, false, false};

volatile long leftTicks = 0;
volatile long rightTicks = 0;
int speedA = 0, speedB = 0;
int timingBudget = 33;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

void setupMotors() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
}

void mspeed(int a, int b) {
  analogWrite(PWMA, abs(a));
  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  analogWrite(PWMB, abs(b));
  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
}

void sendDiagnostics(AsyncWebSocketClient* client) {
  String msg = "";
  for (int i = 0; i < 3; i++) {
    msg += "Sensor " + String(i) + (sensorInit[i] ? ": OK" : ": FAIL") + "\n";
  }
  StaticJsonDocument<128> doc;
  doc["debug"] = msg;
  String out;
  serializeJson(doc, out);
  client->text(out);
}

void sendUpdate() {
  StaticJsonDocument<256> doc;
  doc["lt"] = leftTicks;
  doc["rt"] = rightTicks;
  doc["sl"] = sensorLeft;
  doc["sf"] = sensorFront;
  doc["sr"] = sensorRight;
  String msg;
  serializeJson(doc, msg);
  ws.textAll(msg);
}

void handleWsMessage(void *arg, uint8_t *data, size_t len) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) return;

  if (doc.containsKey("sa")) speedA = constrain(doc["sa"], -255, 255);
  if (doc.containsKey("sb")) speedB = constrain(doc["sb"], -255, 255);
  if (doc.containsKey("tb")) timingBudget = doc["tb"];
  if (doc.containsKey("stop") && doc["stop"] == true) {
    speedA = speedB = 0;
  }
  mspeed(speedA, speedB);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendDiagnostics(client);
  } else if (type == WS_EVT_DATA) {
    handleWsMessage(arg, data, len);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  setupMotors();

  WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  // Simulate sensor init
  sensorInit[0] = false;  // left
  sensorInit[1] = false;  // front
  sensorInit[2] = false; // right (simulate failure)

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 2000) {
    last = millis();
    sendUpdate();
  }
}
