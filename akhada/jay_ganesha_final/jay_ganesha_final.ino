// jay_ganesha_3.ino
// Better front thresh, increased to 120 from 110
#include <Wire.h>
#include <VL53L1X.h>
#include <queue>
#include <utility>
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
int frontThresh = 110;
int baseSpeed = 220;
int rotSpeed = 80; // 130
int minSpeed = 80;

const int MAZESIZE = 16;
int targetX = MAZESIZE - 1, targetY = MAZESIZE - 1;
int blockSize = 27;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2;

// ---------------- PID variables ----------------
float wallKp = 1.4;   // start small
float wallKi = 0.0002; // start near zero
float wallKd = 9;    // start small

unsigned long lastMillis = 0;

void setdelay() {
  lastMillis = millis();
}

void mdelay(unsigned long duration) {
  unsigned long elapsed = millis() - lastMillis;
  if (elapsed < duration) {
    delay(duration - elapsed);
  }
  lastMillis = millis();
}

// ---------------- Distances ----------------
int distLeft, distRight, distFront;
int targetLeftDist = 97, targetRightDist = 97;

const int wf_clearanceThresh = 180;
const int wf_baseSpeed = 255;     // Further reduced for better control
const int wf_targetDist = 80;     // mm desired wall distance
const int wf_frontThresh = 140;   // Reduced threshold for earlier detection
const int wf_wallThresh = 100;    // Increased for better wall detection
const int wf_outerSpeed = 255;    //255 Reduced for smoother turns
const int wf_innerSpeed = 80;     //80 Increased minimum for better movement
const int wf_rotationSpeed = 200; // Further reduced for better accuracy

// ---------------- Wall PID ----------------
float wf_Kp = 1.3;             // Increased for better response
float wf_baseSpeedKi = 0.0002; // Increased for steady-state accuracy
float wf_Kd = 10;            // Increased for stability
float wf_pidPrev = 0, wf_pidInt = 0;

float wallWeight = 0.7f;
float encoderWeight = 0.3f;

bool followLeft = false;
bool followWall = false;

void updateSensors() {
    distLeft = sensor1.read();
    distFront = sensor2.read();
    distRight = sensor3.read();
}

// --- Main forward movement logic ---
float cmToEncoderTicks = 10.3;
const float slowRate = 25;
float encKp=wallKp, encKi=wallKi, encKd=wallKd;
// --- Global PID states (single set) ---
float wallError = 0, wallInt = 0, wallPrev = 0;
float encoderError = 0, encoderInt = 0, encoderPrev = 0;       

void moveForward(int distCm) {
  if (distCm <= 0) return;

  // compute target ticks (long to avoid overflow)
  long targetTicks = (long)roundf(cmToEncoderTicks * (float)distCm);
  if (targetTicks <= 0) return;

  // reset encoders and PID state (ints)
  leftTicks = 0;
  rightTicks = 0;
  wallError = wallInt = wallPrev = 0;
  encoderError = encoderInt = encoderPrev = 0;

    setdelay();
  while (true) {
    int avgTicks = (abs(leftTicks) + abs(rightTicks)) / 2;
    if (avgTicks >= targetTicks) break; // reached goal

    // --- progress & dynamicSpeed (floats) ---
    float progress = (float)avgTicks / (float)targetTicks;        // 0.0 .. 1.0
    // clamp progress
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;

    // exponential slow-down (float)
    float dynamicSpeed = (float)baseSpeed - powf(progress, slowRate) * ((float)baseSpeed - (float)minSpeed);
    // make sure within bounds
    if (dynamicSpeed < (float)minSpeed) dynamicSpeed = (float)minSpeed;
    if (dynamicSpeed > (float)baseSpeed) dynamicSpeed = (float)baseSpeed;

    // --- sensor update & emergency stop ---
    updateSensors();
    if (distFront < frontThresh) break;

    // --- Wall PID (int math) ---
    bool hasLeftWall  = (distLeft  < targetLeftDist + 30);
    bool hasRightWall = (distRight < targetRightDist + 30);

    if (hasLeftWall && hasRightWall) {
      wallError = distLeft - distRight;
    } else if (hasRightWall) {
      wallError = targetRightDist - distRight;
    } else if (hasLeftWall) {
      wallError = distLeft - targetLeftDist;
    } else {
      wallError = 0;
    }
    wallInt += wallError;
    wallInt = constrain(wallInt, -100000, 100000);
    int wallDeriv = wallError - wallPrev;
    wallPrev = wallError;
    int wallPIDValue = wallKp * wallError + wallKi * wallInt + wallKd * wallDeriv;

    // --- Encoder PID (int math) ---
    encoderError = leftTicks - rightTicks;
    encoderInt += encoderError;
    encoderInt = constrain(encoderInt, -100000, 100000);
    int encDeriv = encoderError - encoderPrev;
    encoderPrev = encoderError;
    int encoderPIDValue = encKp * encoderError + encKi * encoderInt + encKd * encDeriv;

    // --- Merge corrections (float totalCorrection) ---
    int totalCorrection = encoderWeight * encoderPIDValue + wallWeight * wallPIDValue;

    // Debug print: two ints, two ints, then two floats
    // Serial.printf("%d %d\t %d %d\t %d\t %f %f\n",
    //               leftTicks, rightTicks, wallPIDValue, encoderPIDValue, totalCorrection, progress, dynamicSpeed);

    // --- apply speeds (convert to int safely) ---
    int leftSpeed  = dynamicSpeed - totalCorrection;
    int rightSpeed = dynamicSpeed + totalCorrection;

    leftSpeed  = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    mspeed(leftSpeed, rightSpeed);

    // mdelay(timingBudget); // loop cadence
  } // while

  // gentle stop
  mspeed(-60, -60);
  delay(2);
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
        b[i] = a[(i - orientation + 4) % 4];
    }

    int type = 8 * b[3] + 4 * b[2] + 2 * b[1] + b[0];
    maze[posX][posY] = type;
    if (posX > 0) {
        maze[posX - 1][posY] |= (b[0] ? 4 : 0); // left wall
    }
    if (posX < MAZESIZE - 1) {
        maze[posX + 1][posY] |= (b[2] ? 1 : 0); // right wall
    }
    if (posY > 0) {
        maze[posX][posY - 1] |= (b[3] ? 2 : 0); // front wall
    }
    if (posY < MAZESIZE - 1) {
        maze[posX][posY + 1] |= (b[1] ? 8 : 0); // back wall
    }
    Serial.printf("(%d, %d) is of type %d\n", posX, posY, maze[posX][posY]);
}

void floodfill() {
    memset(flood, -1, MAZESIZE * MAZESIZE * sizeof(int));
    flood[targetX][targetY] = 0;

    queue<pair<int, int>> q;
    q.push({targetX, targetY});

    while (not q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        if (x > 0 and flood[x - 1][y] == -1 and (maze[x][y] & 1) != 1) {
            flood[x - 1][y] = flood[x][y] + 1;
            q.push({x - 1, y});
        }
        if (x < MAZESIZE - 1 and flood[x + 1][y] == -1 and (maze[x][y] & 4) != 4) {
            flood[x + 1][y] = flood[x][y] + 1;
            q.push({x + 1, y});
            
        }
        if (y > 0 and flood[x][y - 1] == -1 and (maze[x][y] & 8) != 8) {
            flood[x][y - 1] = flood[x][y] + 1;
            q.push({x, y - 1});
        }
        if (y < MAZESIZE - 1 and flood[x][y + 1] == -1 and (maze[x][y] & 2) != 2) {
            flood[x][y + 1] = flood[x][y] + 1;
            q.push({x, y + 1});
        }
    }
}

int rotationDirections[4] = {0, 90, 180, -90}; // 0: forward, 1: right, 2: backward, 3: left
int nextBlock() {
    int target = -1;
    int oldOrient = orientation;
    if (posY < MAZESIZE - 1 and flood[posX][posY + 1] == flood[posX][posY] - 1 and (maze[posX][posY] & 2) != 2) {
        target = 1;
        posY += 1; // Move right
        orientation = 1;
    }
    else if (posX < MAZESIZE - 1 and flood[posX + 1][posY] == flood[posX][posY] - 1 and (maze[posX][posY] & 4) != 4) {
        target = 2;
        posX += 1; // Move down
        orientation = 2;
    }
    else if (posY > 0 and flood[posX][posY - 1] == flood[posX][posY] - 1 and (maze[posX][posY] & 8) != 8) {
        target = 3;
        posY -= 1; // Move left
        orientation = 3;
    }
    else if (posX > 0 and flood[posX - 1][posY] == flood[posX][posY] - 1 and (maze[posX][posY] & 1) != 1) {
        target = 0;
        posX -= 1; // Move up
        orientation = 0;
    }
    if (target == -1)
        return -1;
    return rotationDirections[(orientation - oldOrient + 4) % 4];
}

// ---------------- Rotation with PID ----------------
float encoderCountToDegrees = 0.945;   // calibration factor
//float encoderCountToDegrees = 0.85;   // calibration factor

int rotBaseSpeed = 120;                // base rotation speed
int rotMaxSpeed = 200;                 // clamp for safety

// PID constants (roughly estimated for micromouse 300 rpm motors)
float rotation_kp = 2.0;     // proportional gain
float rotation_ki = 0.002;    // integral gain (small, avoids bias drift)
float rotation_kd = 1.5;     // derivative gain

void rotate(int degree) {
    long targetTicks = encoderCountToDegrees * abs(degree);
    int dir = (degree > 0) ? 1 : -1;

    // Reset encoder counts
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    // Reset PID state
    float error = 0, prevError = 0, integral = 0;
    bool rotatingLeft = true, rotatingRight = true;

    while (rotatingLeft || rotatingRight) {
        // Compute progress of each wheel
        long leftAbs = abs(leftTicks);
        long rightAbs = abs(rightTicks);

        // Stop each motor independently when its target reached
        if (leftAbs >= targetTicks) {
            mspeed(-40, 300);
            rotatingLeft = false;
        }
        if (rightAbs >= targetTicks) {
            mspeed(300, -40);
            rotatingRight = false;
        }
        if (!(rotatingLeft || rotatingRight)) break;

        // PID on difference in encoder counts
        error = (leftAbs - rightAbs);         // balance error
        integral += error;
        integral = constrain(integral, -500, 500);  // anti-windup
        float derivative = error - prevError;
        prevError = error;

        float correction = rotation_kp * error + rotation_ki * integral + rotation_kd * derivative;

        // Apply correction symmetrically
        int leftSpeed  = dir * (rotBaseSpeed - correction);
        int rightSpeed = -dir * (rotBaseSpeed + correction);

        // Constrain speeds
        leftSpeed  = constrain(leftSpeed, -rotMaxSpeed, rotMaxSpeed);
        rightSpeed = constrain(rightSpeed, -rotMaxSpeed, rotMaxSpeed);

        mspeed(leftSpeed, rightSpeed);
    }

    // Small counter-brake to rotation_kill inertia
    mspeed(-dir * 80, dir * 80);
    delay(5);
    mspeed(0, 0);
    Serial.printf("Target was %d, it traveled %d %d \n", targetTicks, leftTicks, rightTicks);
}

// ---------------- PID routines ----------------
void runWallPIDSingle(int measured, int desired, bool invertMirror) {
    // invertMirror == true -> mirror error (for right-wall case) so code below can be identical
    float e = (float)measured - (float)desired;
    if (invertMirror)
        e = -e;

    wf_pidInt += e;
    wf_pidInt = constrain(wf_pidInt, -5000, 5000); // Increased integral limits
    float deriv = e - wf_pidPrev;
    wf_pidPrev = e;

    float pid = wf_Kp * e + wf_baseSpeedKi * wf_pidInt + wf_Kd * deriv;

    // pid positive -> steering correction
    int leftSp = constrain(wf_baseSpeed - pid, 30, 255);  // Added minimum speed
    int rightSp = constrain(wf_baseSpeed + pid, 30, 255); // Added minimum speed

    mspeed(leftSp, rightSp);
}

// ---------------- Main behavior (symmetric) ----------------
void behaviorStep() {
    updateSensors();
    // 1) Handle missing wall first - search behavior (higher priority than front obstacle)
    if (followLeft) {
        if (distLeft > wf_wallThresh) {
            mspeed(wf_innerSpeed, wf_outerSpeed); // Turn left to find wall
            return;
        }
    }
    else {
        if (distRight > wf_wallThresh) {
            mspeed(wf_outerSpeed, wf_innerSpeed); // Turn right to find wall
            return;
        }
    }

    if (distFront < wf_frontThresh) {
        Serial.println("Front obstacle detected - executing turn");

        mspeed(-100, -100);
        delay(1);

        setdelay();
        while (distFront < wf_clearanceThresh) {
            if (followLeft) {
                mspeed(wf_rotationSpeed, -wf_rotationSpeed);
            }
            else {
                mspeed(-wf_rotationSpeed, wf_rotationSpeed);
            }
            mdelay(timingBudget);
            updateSensors();
        }
        if (followLeft) mspeed(-80, 80);
        else mspeed(80, -80);
        delay(2);
        mspeed(0, 0);
        return;
    }

    if (followLeft) {
        runWallPIDSingle(distLeft, wf_targetDist, false);
    }
    else {
        runWallPIDSingle(distRight, wf_targetDist, true);
    }
}

void setupSensors() {
    Wire.begin(33, 32);
    Wire.setClock(400000);

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
        while (1)
            ;
    }
    sensor1.setAddress(ADDR1);

    // --- Step 3: Boot sensor2 ---
    pinMode(XSHUT2, INPUT);
    delay(10);
    if (!sensor2.init()) {
        Serial.println("Sensor2 init failed!");
        while (1)
            ;
    }
    sensor2.setAddress(ADDR2);

    // --- Step 4: Boot sensor3 ---
    pinMode(XSHUT3, INPUT);
    delay(10);
    if (!sensor3.init()) {
        Serial.println("Sensor3 init failed!");
        while (1)
            ;
    }
    sensor3.setAddress(ADDR3);

    // --- Step 5: Configure all sensors ---
    sensor1.setDistanceMode(VL53L1X::Medium);
    sensor1.setMeasurementTimingBudget(timingBudget * 1000); // 22 ms
    sensor1.startContinuous(timingBudget);

    sensor2.setDistanceMode(VL53L1X::Medium);
    sensor2.setMeasurementTimingBudget(timingBudget * 1000);
    sensor2.startContinuous(timingBudget);

    sensor3.setDistanceMode(VL53L1X::Medium);
    sensor3.setMeasurementTimingBudget(timingBudget * 1000);
    sensor3.startContinuous(timingBudget);
    Serial.println("All sensors initialized.");
}

void setupMotorEncodersMore() {
    // Motor setup
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

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
}

void calibrate() {
    // calibrate sensors at the start by taking 10 readings
    int epoch = 10;
    targetLeftDist = targetRightDist = 0;

    setdelay();
    for (int i = 0; i < epoch; i++) {
        targetLeftDist += sensor1.read();
        targetRightDist += sensor3.read();
        mdelay(timingBudget);
    }
    targetLeftDist /= epoch;
    targetRightDist /= epoch;

    delay(1000);
    rotate(45);
    rotate(-45);
}

void waitButtonPress() {
    while (1) {
        if (digitalRead(BTN1) == HIGH) {
            followWall = true;
            break;
        }
        else if (digitalRead(BTN2) == HIGH) {
            followWall = false;
            break;
        }
        delay(50);
        digitalWrite(LED1, LOW);
        delay(50);
        digitalWrite(LED1, HIGH);
    }
    delay(500);
    while (1) {
        if (digitalRead(BTN1) == HIGH) {
            followLeft = true;
            break;
        }
        else if (digitalRead(BTN2) == HIGH) {
            followLeft = false;
            break;
        }
        delay(50);
        digitalWrite(LED2, LOW);
        delay(50);
        digitalWrite(LED2, HIGH);
    }
}

void setup() {
    Serial.begin(115200);
    // Use custom I2C pins: SDA = 33, SCL = 32
    setupSensors();

    setupMotorEncodersMore();

    calibrate();

    Serial.println("Waiting for button press...");
    waitButtonPress();

    Serial.println("Starting");
    delay(1000);

    // while (1) {
    //     if (digitalRead(BTN1) == 1) {
    //         delay(500);
    //         rotate(180);
    //         delay(500);
    //     }
    // }

    // while (1) {
    //     if (digitalRead(BTN1) == HIGH) {
    //         rotate(180);
    //         delay(500);
    //     }
    // } 

    // while(1) {
    //     moveForward(25);
    //     delay(500);
    // }

    // while (1) {
    //     // rotate(90);
    //     // delay(1000);
    //     // rotate(180);
    //     // delay(1000);
    //     // rotate(-90);
    //     // delay(1000);
    //     rotate(360);
    //     delay(3000);
    // }

    // while (1) {
    //     if (digitalRead(BTN1) == HIGH) {
    //     delay(500);
    //     rotate(90);
    //     }
    //     else if (digitalRead(BTN2) == HIGH) {
    //         delay(500);
    //         rotate(-180);
    //     }
    // }
}

int optimise_run = false;
void mazeSolver() {
    while (1) {
        identifyBlock();
        floodfill();
        int degrees = nextBlock();
        while (degrees == -1) {
            Serial.println("End of the maze");
            // for (int i = 0; i < 20; i++) {
            //     digitalWrite(LED1, HIGH);
            //     digitalWrite(LED2, HIGH);
            //     delay(50);
            //     digitalWrite(LED1, LOW);
            //     digitalWrite(LED2, LOW);
            //     delay(50);
            // }
            followWall = true;
            delay(100);
            return;
            // if (!optimise_run) {
            //     targetX=0;
            //     targetY=0;
            // } else {
            //     targetX=MAZESIZE-1;
            //     targetY=MAZESIZE-1;
            // }
            break;
        }
        rotate(degrees);
        delay(100);
        moveForward(blockSize);
        delay(100);
    }
}

void wallFollower() {
    setdelay();
    while(1) {
        behaviorStep();
        mdelay(timingBudget);
        if (digitalRead(BTN1) == 1 or digitalRead(BTN2) == 1) {
            mspeed(0, 0);
            delay(500);
            waitButtonPress();
            delay(500);
            posX = 0;
            posY = 0;
            orientation = 2;
            return;
        }
    }
}

void normalTurn () {
    moveForward(blockSize);
    moveForward(blockSize);
    moveForward(blockSize);
    moveForward(blockSize);
    moveForward(blockSize);
    moveForward(blockSize);
    delay(100);
    rotate(-90);
    moveForward(blockSize);
    followLeft = false;
    followWall = true;
    return;
}

void loop() {
    // Serial.printf("Follow Wall: %d, Follow Left: %d \n", followWall, followLeft);
    // delay(1000);
    if (followWall) {
        digitalWrite(LED1, followLeft);
        digitalWrite(LED2, !followLeft);
        wallFollower();
    } else {
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        // mazeSolver();
        normalTurn();

    }
}

void mspeed(int a, int b) {
    if (abs(a) <= 255) {
        digitalWrite(AIN1, a >= 0);
        digitalWrite(AIN2, a < 0);
        analogWrite(PWMA, abs(a));
    }

    if (abs(b) <= 255) {
        digitalWrite(BIN1, b >= 0);
        digitalWrite(BIN2, b < 0);
        analogWrite(PWMB, abs(b));
    }
}