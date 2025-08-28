#include <Wire.h>
#include <VL53L1X.h>
#include <queue>
#include <utility>
#include "driver/pcnt.h"
#include "wifilogger.h"
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

#define PCNT_LIM_VAL 30000

int16_t leftTicks = 0;
int16_t rightTicks = 0;

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

int baseSpeed = 240;

const int MAZESIZE = 5;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; 
int orientation = 2; 

int dummymaze[MAZESIZE][MAZESIZE];

// ---------------- PID variables ----------------
float wallKp = 1.45;   // start small
float wallKi = 0.0001;  // start near zero
float wallKd = 0.93;   // start small

float wallError = 0;
float wallPrev  = 0;
float wallInt   = 0;
float wallPIDValue = 0;

void resetEncoders() {
  pcnt_counter_clear(PCNT_UNIT_0); // Left encoder
  pcnt_counter_clear(PCNT_UNIT_1); // Right encoder
}

void readEncoders() {
  pcnt_get_counter_value(PCNT_UNIT_0, &leftTicks);
  pcnt_get_counter_value(PCNT_UNIT_1, &rightTicks);
}


// ---------------- Distances ----------------
int distLeft, distRight, distFront;

void updateSensors() {
  distLeft  = sensor1.read();
  distFront = sensor2.read();
  distRight = sensor3.read();
}

// ---------------- Both-Wall PID ----------------
void runWallPID(int dL, int dR) {
  wallError = dL - dR;               // error = difference
  wallInt += wallError;              // accumulate
  wallInt = constrain(wallInt, -1000, 1000); // anti-windup
  float wallDeriv = wallError - wallPrev;
  wallPrev = wallError;

  wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;

  int leftSpeed  = constrain(baseSpeed - wallPIDValue, 0, 255);
  int rightSpeed = constrain(baseSpeed + wallPIDValue, 0, 255);

  mspeed(leftSpeed, rightSpeed);
}


int targetRightDist = 80; // mm
void runRightWallPID(int dR) {
  int error =  targetRightDist - dR;         // error = actual - desired distance
  wallInt += error;                        // integral term
  wallInt = constrain(wallInt, -1000, 1000); // anti-windup
  float deriv = error - wallPrev;          // derivative term
  wallPrev = error;

  float pidVal = wallKp * error + wallKi * wallInt + wallKd * deriv;

  int leftSpeed  = constrain(baseSpeed - pidVal, 0, 255);  // slow down if too close
  int rightSpeed = constrain(baseSpeed + pidVal, 0, 255);  // speed up opposite wheel

  mspeed(leftSpeed, rightSpeed);

  // Debug
}


int targetLeftDist = 80; // mm
void runLeftWallPID(int dL) {
  int error = dL - targetLeftDist;         // error = actual - desired distance
  wallInt += error;                        // integral term
  wallInt = constrain(wallInt, -1000, 1000); // anti-windup
  float deriv = error - wallPrev;          // derivative term
  wallPrev = error;

  float pidVal = wallKp * error + wallKi * wallInt + wallKd * deriv;

  int leftSpeed  = constrain(baseSpeed - pidVal, 0, 255);  // slow down if too close
  int rightSpeed = constrain(baseSpeed + pidVal, 0, 255);  // speed up opposite wheel

  mspeed(leftSpeed, rightSpeed);

  // Debug
}

void runEncoderPID() {
  wallError = leftTicks - rightTicks;               // error = difference
  wallInt += wallError;              // accumulate
  wallInt = constrain(wallInt, -1000, 1000); // anti-windup
  float wallDeriv = wallError - wallPrev;
  wallPrev = wallError;

  wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;

  int leftSpeed  = constrain(baseSpeed - wallPIDValue, 0, 255);
  int rightSpeed = constrain(baseSpeed + wallPIDValue, 0, 255);

  mspeed(leftSpeed, rightSpeed);
}


float cmToEncoderTicks = 19;
void moveForward() {
    int target = cmToEncoderTicks * 25; // 25 cm
    resetEncoders();
    int thresh = 150;
    int mode = 0, newMode = 0;
    wallInt=0;
    Serial.printf("Moving forward 25 cm, target %d\n", target);
        // Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
    while (abs(leftTicks) < target or abs(rightTicks) < target) {
        updateSensors();
        readEncoders();
        // Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
        // runWallPID(distLeft, distRight);
        float isWallLeft = (distLeft < thresh);
        float isWallRight = (distRight < thresh);
        newMode = isWallLeft*2+isWallRight;
        if (mode != newMode) wallInt=0;
        mode = newMode;
        if (isWallLeft and isWallRight) {
            runWallPID(distLeft, distRight);
            // Serial.println("Both walls");
        } else if (isWallLeft) {
            runLeftWallPID(distLeft);
            // Serial.println("Left walls");
        } else if (isWallRight) {
            runRightWallPID(distRight);
            // Serial.println("Right wall");
        } else {
            runEncoderPID();
            // Serial.println("Encoder PID");
        }
        delay(timingBudget);
    }
    mspeed(0, 0);
}

void identifyBlock() {
    uint8_t a[4], b[4];
    int thresh = 140;
    
    a[0] = distFront < thresh;
    a[1] = distRight < thresh;
    a[3] = distLeft < thresh;
    a[2] = 0;

    for (int i = 0; i < 4; i++) {
        b[i] = a[(i - orientation+4) % 4];
    }

    int type = 8 * b[3] + 4 * b[2] + 2 * b[1] + b[0];
    maze[posX][posY] = type;
    if (posX > 0) {
        maze[posX-1][posY] |= (b[0] ? 4 : 0); // top wall
    }
    if (posX < MAZESIZE - 1) {
        maze[posX+1][posY] |= (b[2] ? 1 : 0); // bottom wall
    }
    if (posY > 0) {
        maze[posX][posY-1] |= (b[3] ? 2 : 0); // left wall
    }
    if (posY < MAZESIZE - 1) {
        maze[posX][posY+1] |= (b[1] ? 8 : 0); // right wall
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

// float ticksPerDegree = 0.945;  
float ticksPerDegree = 2.43;

float Kp_turn = 2.0, Ki_turn = 0.0, Kd_turn = 0.5;

void rotate(int degree) {
    if (degree == 0) return;

    resetEncoders(); 
    
    long targetTicks = abs(degree) * ticksPerDegree;
    int dir = (degree > 0) ? 1 : -1;

    // --- LOGIC FIX 1: Flags to stop each motor independently ---
    bool leftDone = false;
    bool rightDone = false;

    // PID state variables
    float integralL = 0, integralR = 0;
    float prevErrorL = 0, prevErrorR = 0;
    
    // The loop continues as long as AT LEAST ONE motor is not done.
    while (!leftDone || !rightDone) {
        readEncoders();

        long errorL = targetTicks - abs(leftTicks);
        long errorR = targetTicks - abs(rightTicks);

        // Check if motors have reached their target range
        if (abs(errorL) < 5) leftDone = true;
        if (abs(errorR) < 5) rightDone = true;

        int outputL = 0;
        int outputR = 0;

        // --- Left Motor PID ---
        if (!leftDone) {
            integralL += errorL;
            integralL = constrain(integralL, -2000, 2000);
            float derivL = errorL - prevErrorL;
            prevErrorL = errorL;
            outputL = Kp_turn * errorL + Ki_turn * integralL + Kd_turn * derivL;
        }

        // --- Right Motor PID ---
        if (!rightDone) {
            integralR += errorR;
            integralR = constrain(integralR, -2000, 2000);
            float derivR = errorR - prevErrorR;
            prevErrorR = errorR;
            outputR = Kp_turn * errorR + Ki_turn * integralR + Kd_turn * derivR;
        }

        // --- LOGIC FIX 2: Reduce max speed when close to the target ---
        int maxSpeedL = (abs(errorL) < 50) ? 80 : 255;
        int maxSpeedR = (abs(errorR) < 50) ? 80 : 255;
        
        int minSpeed = 60; // Minimum speed to overcome inertia

        // Constrain motor outputs
        if (outputL != 0) outputL = constrain(outputL, minSpeed, maxSpeedL);
        if (outputR != 0) outputR = constrain(outputR, minSpeed, maxSpeedR);
        
        // Send a single motor command
        mspeed(dir * outputL, -dir * outputR);

        delay(10);
    }

    mspeed(0, 0); // Ensure motors are stopped
    delay(100);   // Settle time
}


// void rotate(int degrees) {
//   resetEncoders();
//   long targetTicks = abs(degrees) * ticksPerDegree;
//   int dir = (degrees > 0) ? 1 : -1;  

//   float integralR = 0;
//   float lastError = 0;


//   while (true) {
//     // For rotation: left should go +target, right should go -target
//     long leftTarget  = targetTicks;
//     long rightTarget = targetTicks;

//     readEncoders();
//     long leftError  = leftTarget  - abs(leftTicks);
//     long rightError = rightTarget - abs(rightTicks);

//     // Stop condition: both motors close enough
//     if (abs(leftError) < 5 && abs(rightError) < 5) break;

//     // PID for left motor
//     integralR += leftError;
//     float diffL = leftError - lastError;
//     float leftOutput = rotationKp * leftError + rotationKi * integralR + rotationKd * diffL;

//     // PID for right motor
//     integralR += rightError;
//     float diffR = rightError - lastError;
//     float rightOutput = rotationKp * rightError + rotationKi * integralR + rotationKd * diffR;

//     lastError = (leftError + rightError) / 2;

//     // Clamp & add deadzone compensation
//     if (abs(leftOutput) < 80) leftOutput = (leftOutput >= 0 ? 80 : -80);
//     if (abs(rightOutput) < 80) rightOutput = (rightOutput >= 0 ? 80 : -80);
//     leftOutput  = constrain(leftOutput,  -255, 255);
//     rightOutput = constrain(rightOutput, -255, 255);

//     // Apply: left forward, right backward
//     mspeed(dir * leftOutput, -dir * rightOutput);
//     delay(20);
//   }

//   mspeed(0, 0);
//   delay(500);
// }

void setupPCNT(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = pinA;
  pcnt_config.ctrl_gpio_num = pinB;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = unit;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DEC;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = PCNT_LIM_VAL;
  pcnt_config.counter_l_lim = -PCNT_LIM_VAL;

  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
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

  
  setupPCNT(PCNT_UNIT_0, LEFT_ENC_A, LEFT_ENC_B);
  setupPCNT(PCNT_UNIT_1, RIGHT_ENC_A, RIGHT_ENC_B);
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
    rotate(0);

    initWiFiLogger("MeOrYou", "12345678");
    log("Logger working");
    
    rotate(45);

    Serial.println("Waiting for button press...");
    while (digitalRead(BTN1) == LOW) {}
    delay(1000);
}

void logMaze() {
    String s = "Maze:\n";
    for (int i = 0; i < MAZESIZE; i++) {
        for (int j = 0; j < MAZESIZE; j++) {
            s += String(maze[i][j]) + " ";
        }
        s += "\n";
    }
    log(s);

    // s = "Flood:\n";
    // for (int i = 0; i < MAZESIZE; i++) {
    //     for (int j = 0; j < MAZESIZE; j++) {
    //         s += String(flood[i][j]) + " ";
    //     }
    //     s += "\n";
    // }
    // log(s);
}

void logSensors() {
    String s = "Sensors: L=" + String(distLeft) + " F=" + String(distFront) + " R=" + String(distRight);
    log(s);
}

void loop() {
    // delay(200);
    // updateSensors();
    // logSensors();
    // Serial.printf("Bot at (%d, %d), orient: %d \n", posX, posY, orientation);
    log("Bot at (" + String(posX) + ", " + String(posY) + "), orient: " + String(orientation));
    updateSensors();
    logSensors();
    identifyBlock();
    logMaze();
    floodfill();
    int degrees = nextBlock();
    while (degrees == -1) {
        Serial.println("End of the maze");
        delay(1000);
    }
    // log("Rotating " + String(degrees) + " degrees");
    rotate(degrees);
    // log("Rotation done");
    delay(200);
    // log("Moving forward");
    moveForward();
    delay(200);

    // updateSensors();
    // Serial.printf("%d %d %d \n", distLeft, distFront, distRight);
    // delay(200);
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