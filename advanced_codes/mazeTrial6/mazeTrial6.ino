#include <Wire.h>
#include <VL53L1X.h>
#include <queue>
#include <utility>
using namespace std;

#define BTN1 34
#define BTN2 35

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

int timingBudget = 20; // in ms

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

int baseSpeed = 240;

const int MAZESIZE = 5;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2;


// ---------------- PID variables ----------------
float wallKp = 1.6;   // start small
float wallKi = 0.0002;  // start near zero
float wallKd = 0.65;   // start small


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
float cmToEncoderTicks = 9.6;
void moveForward() {
    int target = cmToEncoderTicks * 25; // 25 cm
    leftTicks = rightTicks = 0;
   
    int thresh = 130;

    // Reset all PID states before starting a new movement.
    bothWallState = {};
    rightWallState = {};
    leftWallState = {};
    encoderState = {};

    // 💡 IMPROVEMENT 3: Loop until the AVERAGE distance is met.
    while ((abs(leftTicks) + abs(rightTicks)) / 2 < target) {
        updateSensors();

        bool isWallLeft = (distLeft < thresh);
        bool isWallRight = (distRight < thresh);

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
    mspeed(0, 0);
}

void identifyBlock() {
    uint8_t a[4], b[4];
    int thresh = 150;
   
    a[0] = sensor2.read() < thresh;
    a[1] = sensor3.read() < thresh;
    a[3] = sensor1.read() < thresh;
    a[2] = 0;

    for (int i = 0; i < 4; i++) {
        b[i] = a[(i - orientation+4) % 4];
    }

    int type = 8 * b[3] + 4 * b[2] + 2 * b[1] + b[0];
    maze[posX][posY] = type;
    if (posX > 0) {
        maze[posX-1][posY] |= (b[0] ? 4 : 0); // left wall
    }
    if (posX < MAZESIZE - 1) {
        maze[posX+1][posY] |= (b[2] ? 1 : 0); // right wall
    }
    if (posY > 0) {
        maze[posX][posY-1] |= (b[3] ? 2 : 0); // front wall
    }
    if (posY < MAZESIZE - 1) {
        maze[posX][posY+1] |= (b[1] ? 8 : 0); // back wall
    }
    Serial.printf("(%d, %d) is of type %d\n", posX, posY, maze[posX][posY]);
}

void floodfill() {
    int targetX = MAZESIZE - 1, targetY = MAZESIZE - 1;
    memset(flood, -1, MAZESIZE* MAZESIZE * sizeof(int));
    flood[targetX][targetY] = 0;

    queue<pair<int, int>> q;
    q.push({targetX, targetY});

    while (not q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        if (x > 0 and flood[x-1][y] == -1 and (maze[x][y]&1) != 1) {
            flood[x-1][y] = flood[x][y] + 1;
            q.push({x-1, y});
        }
        if (x < MAZESIZE - 1 and flood[x+1][y] == -1 and (maze[x][y]&4) != 4) {
            flood[x+1][y] = flood[x][y] + 1;
            q.push({x+1, y});
        }
        if (y > 0 and flood[x][y-1] == -1 and (maze[x][y]&8) != 8) {
            flood[x][y-1] = flood[x][y] + 1;
            q.push({x, y-1});
        }
        if (y < MAZESIZE - 1 and flood[x][y+1] == -1 and (maze[x][y]&2) != 2) {
            flood[x][y+1] = flood[x][y] + 1;
            q.push({x, y+1});
        }
    }
}

int rotationDirections[4] = { 0, 90, 180, -90 }; // 0: forward, 1: right, 2: backward, 3: left
int nextBlock() {
    int target = -1;
    int oldOrient = orientation;
    if (posX > 0 and flood[posX-1][posY] == flood[posX][posY] - 1) {
        target = 0;
        posX -= 1; // Move left
        orientation = 0;
    } else if (posX < MAZESIZE - 1 and flood[posX+1][posY] == flood[posX][posY] - 1) {
        target = 2;
        posX += 1; // Move right
        orientation = 2;
    } else if (posY > 0 and flood[posX][posY-1] == flood[posX][posY] - 1) {
        target = 3;
        posY -= 1; // Move forward
        orientation = 3;
    } else if (posY < MAZESIZE - 1 and flood[posX][posY+1] == flood[posX][posY] - 1) {
        target = 1;
        posY += 1; // Move backward
        orientation = 1;
    }
    if (target == -1) return -1;
    return rotationDirections[(orientation - oldOrient + 4) % 4];
}

float encoderCountToDegrees = 0.945;
int dynSpeed = 80;
void rotate(int degree) {
    int target = encoderCountToDegrees * abs(degree);
    int dir = degree > 0 ? 1 : -1;
    bool rotatingLeft = true, rotatingRight = true;
    leftTicks = rightTicks = 0;

    mspeed(dir*baseSpeed, -dir*baseSpeed);
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
    mspeed(0, 0);
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
    while (digitalRead(BTN1) == LOW) {}
    delay(1000);
}

void loop() {
    // moveForward();
    // delay(2000);
    // updateSensors();
    // runLeftWallPID(distLeft);
    // runRightWallPID(distRight);
    // runEncoderPID();
    // runWallPID(distLeft, distRight);
    // delay(20);

    Serial.printf("Sensor 1: %d \t Sensor 2: %d \t Sensor 3: %d\n", distLeft, distFront, distRight);
    delay(200);
    Serial.printf("Bot at (%d, %d), orient: %d \n", posX, posY, orientation);
    identifyBlock();
    floodfill();
    int degrees = nextBlock();
    while (degrees == -1) {
        Serial.println("End of the maze");
        delay(1000);
    }
    rotate(degrees);
    delay(200);
    moveForward();
    delay(200);
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