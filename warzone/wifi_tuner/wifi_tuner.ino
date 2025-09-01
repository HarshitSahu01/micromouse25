#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
using namespace std;

#define BTN1 34
#define BTN2 35

#define LED1 0
#define LED2 12

#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

// Encoder pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

struct PIDControllerState {
    float integral;
    float prevError;

    PIDControllerState() : integral(0.0), prevError(0.0) {}
};

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// XSHUT pins
#define XSHUT1 13
#define XSHUT2 27
#define XSHUT3 25

// I2C addresses we’ll assign
#define ADDR1 0x30
#define ADDR2 0x31
#define ADDR3 0x32

// Sensor objects
VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;


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

int timingBudget = 20; // in mm
int frontThresh = 60;
int baseSpeed = 240;
int rotSpeed = 150;

const int MAZESIZE = 8;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2;


// ---------------- PID variables ----------------
float wallKp = 1.2;   // start small
float wallKi = 0.0002;  // start near zero
float wallKd = 10.2;   // start small


// ---------------- Distances ----------------
int distLeft, distRight, distFront;
int targetLeftDist, targetRightDist;

void updateSensors() {
  distLeft  = sensor1.read();
  distFront = sensor2.read();
  distRight = sensor3.read();
}
// 💡 IMPROVEMENT 1: Create a structure to hold PID

// --- Instantiate a separate state for each control mode ---
PIDControllerState bothWallState;
PIDControllerState rightWallState;
PIDControllerState leftWallState;
PIDControllerState encoderState;

// --- Refactored PID functions now take their state as an argument ---

void runWallPID(int dL, int dR, PIDControllerState& state) {
    float error = dL - dR; // Try to keep this difference zero (centered)
    state.integral += error;
    state.integral = constrain(state.integral, -1000, 1000);
    float derivative = error - state.prevError;
    state.prevError = error;

    float pidOutput = wallKp * error + wallKi * state.integral + wallKd * derivative;

    int leftSpeed  = constrain(baseSpeed - pidOutput, 0, 255);
    int rightSpeed = constrain(baseSpeed + pidOutput, 0, 255);
    mspeed(leftSpeed, rightSpeed);
}

void runRightWallPID(int dR, int targetDist, PIDControllerState& state) {
    float error = targetDist - dR; // If dR is too big, error is negative -> turn right
    state.integral += error;
    state.integral = constrain(state.integral, -1000, 1000);
    float derivative = error - state.prevError;
    state.prevError = error;

    float pidOutput = wallKp * error + wallKi * state.integral + wallKd * derivative;

    int leftSpeed  = constrain(baseSpeed - pidOutput, 0, 255);
    int rightSpeed = constrain(baseSpeed + pidOutput, 0, 255);
    mspeed(leftSpeed, rightSpeed);
}

void runLeftWallPID(int dL, int targetDist, PIDControllerState& state) {
    // BUG FIX: Error calculation is now inverted.
    // If dL is too big, error is negative -> turn right (towards wall).
    float error = targetDist - dL;
    state.integral += error;
    state.integral = constrain(state.integral, -1000, 1000);
    float derivative = error - state.prevError;
    state.prevError = error;

    float pidOutput = wallKp * error + wallKi * state.integral + wallKd * derivative;

    // For left wall, a positive error (too close) should make us turn right.
    // Turning right means speeding up left wheel and slowing down right.
    int leftSpeed  = constrain(baseSpeed + pidOutput, 0, 255);
    int rightSpeed = constrain(baseSpeed - pidOutput, 0, 255);
    mspeed(leftSpeed, rightSpeed);
}

void runEncoderPID(PIDControllerState& state) {
    float error = leftTicks - rightTicks; // Keep encoder difference at zero
    state.integral += error;
    state.integral = constrain(state.integral, -1000, 1000);
    float derivative = error - state.prevError;
    state.prevError = error;

    float pidOutput = wallKp * error + wallKi * state.integral + wallKd * derivative;

    int leftSpeed  = constrain(baseSpeed - pidOutput, 0, 255);
    int rightSpeed = constrain(baseSpeed + pidOutput, 0, 255);
    mspeed(leftSpeed, rightSpeed);
}


// --- Main forward movement logic ---
float cmToEncoderTicks = 10.3;
void moveForward() {
    int target = cmToEncoderTicks * 25; // 25 cm
    leftTicks = rightTicks = 0;
   int error = 10;

    // Reset all PID states before starting a new movement.
    bothWallState = {};
    rightWallState = {};
    leftWallState = {};
    encoderState = {};

    // 💡 IMPROVEMENT 3: Loop until the AVERAGE distance is met.
    while ((abs(leftTicks) + abs(rightTicks)) / 2 < target) {
        updateSensors();
        if (distFront < frontThresh) break;

        bool isWallLeft = (distLeft < targetLeftDist + error);
        bool isWallRight = (distRight < targetRightDist + error);

        if (isWallLeft && isWallRight) {
            runWallPID(distLeft, distRight, bothWallState);
        } else if (isWallLeft) {
            runLeftWallPID(distLeft, targetLeftDist, leftWallState);
        } else if (isWallRight) {
            runRightWallPID(distRight, targetRightDist, rightWallState);
        } else {
            runEncoderPID(encoderState);
        }
        delay(20); // A small delay is good for PID stability
    }
    mspeed(-50, -50);
    delay(2);
    mspeed(0, 0);
}
// float encoderCountToDegrees = 0.945;
float encoderCountToDegrees = 0.945;
int dynSpeed = 80;
void rotate(int degree) {
    int target = encoderCountToDegrees * abs(degree);
    int dir = degree > 0 ? 1 : -1;
    bool rotatingLeft = true, rotatingRight = true;
    leftTicks = rightTicks = 0;

    mspeed(dir*rotSpeed, -dir*rotSpeed);
    while (rotatingLeft or rotatingRight) {
        if (abs(leftTicks) > target) {
            mspeed(0, 300);
            rotatingLeft = false;
        }
        if (abs(rightTicks) > target) {
            mspeed(300, 0);
            rotatingRight = false;
        }
        delay(5);
    }
    mspeed(dir*50, -dir*50);
    delay(2);
    mspeed(0, 0);
}

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


bool clientConnected = false;
bool running = false;
float targetDistance = 0.0;

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

void setup() {
    Serial.begin(115200);
    // Use custom I2C pins: SDA = 33, SCL = 32
  Wire.begin(33, 32);

  // --- Step 1: Reset all sensors ---
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(100);

  // --- Step 2: Boot sensor1 only ---
  pinMode(XSHUT1, INPUT); // release -> pulled HIGH
  delay(10);
  if (!sensor1.init()) {
    Serial.println("Sensor1 init failed!");
    while (1);
  }
  sensor1.setAddress(ADDR1);

  // --- Step 3: Boot sensor2 ---
  pinMode(XSHUT2, INPUT);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Sensor2 init failed!");
    while (1);
  }
  sensor2.setAddress(ADDR2);

  // --- Step 4: Boot sensor3 ---
  pinMode(XSHUT3, INPUT);
  delay(10);
  if (!sensor3.init()) {
    Serial.println("Sensor3 init failed!");
    while (1);
  }
  sensor3.setAddress(ADDR3);

  // --- Step 5: Configure all sensors ---
  sensor1.setDistanceMode(VL53L1X::Medium);
  sensor1.setMeasurementTimingBudget(timingBudget*1000); // 22 ms
  sensor1.startContinuous(timingBudget);

  sensor2.setDistanceMode(VL53L1X::Medium);
  sensor2.setMeasurementTimingBudget(timingBudget*1000);
  sensor2.startContinuous(timingBudget);

  sensor3.setDistanceMode(VL53L1X::Medium);
  sensor3.setMeasurementTimingBudget(timingBudget*1000);
  sensor3.startContinuous(timingBudget);
  Serial.println("All sensors initialized.");
    // Motor setup
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Encoder setup
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  Serial.println("Motor Encoders initialised");

  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

    // calibrate sensors at the start by taking 10 readings
    int epoch=10;
    targetLeftDist = targetRightDist = 0;

    for (int i = 0; i < epoch; i++) {
        targetLeftDist += sensor1.read();
        targetRightDist += sensor3.read();
        delay(timingBudget);
    }
    targetLeftDist /= epoch;
    targetRightDist /= epoch;

    delay(1000);
    rotate(45);
    rotate(-45);

    
    Serial.println("Waiting for button press...");
    while (digitalRead(BTN1) == LOW) {delay(5);}
    Serial.println("Starting");
    delay(1000);

    Serial.println("Sensors initialized. Running Both-Wall PID only.");

    WiFi.begin("MeOrYou", "12345678");
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  if (!MDNS.begin("esp32")) Serial.println("mDNS init failed");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

}

void loop() {

    // Serial.printf("Sensor 1: %d \t Sensor 2: %d \t Sensor 3: %d\n", distLeft, distFront, distRight);
    // delay(200);
    // Serial.printf("Bot at (%d, %d), orient: %d \n", posX, posY, orientation);
    moveForward();
    delay(10);
}

void mspeed(int a, int b) {
    if (abs(a) <= 255) {
        digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
        analogWrite(PWMA, abs(a));
    }

    if (abs(b) <= 255) {
        digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
        analogWrite(PWMB, abs(b));
    }
}