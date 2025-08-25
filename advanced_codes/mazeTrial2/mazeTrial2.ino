#include <Wire.h>
#include <VL53L1X.h>
#include <queue>
#include <utility>
using namespace std;

// Utility function
void print() {
  Serial.println();  // Print a newline at the end of the line
}

// Variadic template function to handle multiple arguments
template<typename T, typename... Args>
void print(T first, Args... args) {
  Serial.print(first);
  Serial.print(" ");
  print(args...);  // Recursive call to handle the next argument
}


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

int baseSpeed = 150;

const int MAZESIZE = 5;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2; 

// int ardmaze[MAZESIZE][MAZESIZE] = {
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0
// };
int dummymaze[MAZESIZE][MAZESIZE] = {
    11, 13, 3, 9, 3,
    8, 1, 0, 4, 6,
    10, 10, 8, 5, 3,
    10, 8, 0, 3, 10,
    14, 12, 6, 14, 14
};

// ---------------- PID variables ----------------
float wallKp = 1.25;   // start small
float wallKi = 0.001;  // start near zero
float wallKd = 0.255;   // start small

float wallError = 0;
float wallPrev  = 0;
float wallInt   = 0;
float wallPIDValue = 0;


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


float cmToEncoderTicks = 11.43;
void moveForward() {
    int target = cmToEncoderTicks * 25; // 25 cm
    leftTicks = rightTicks = 0;
    int thresh = 130;
    int mode = 0, newMode = 0;
    wallInt=0;
    print("Moving forward 25 cm, target ", target);
        // Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
    while (abs(leftTicks) < target and abs(rightTicks) < target) {
        updateSensors();
        // Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
        // runWallPID(distLeft, distRight);
        float isWallLeft = (distLeft < thresh);
        float isWallRight = (distRight < thresh);
        newMode = isWallLeft*2+isWallRight;
        if (mode != newMode) wallInt=0;
        mode = newMode;
        if (isWallLeft and isWallRight) {
            runWallPID(distLeft, distRight);
            Serial.println("Both walls");
        } else if (isWallLeft) {
            runLeftWallPID(distLeft);
            Serial.println("Left walls");
        } else if (isWallRight) {
            runRightWallPID(distRight);
            Serial.println("Right wall");
        } else {
            runEncoderPID();
            Serial.println("Encoder PID");
        }
        delay(timingBudget);
    }
    mspeed(0, 0);
}

// void moveForward() {
//     int target = cmToEncoderTicks * 25; // 25 cm
//     leftTicks = rightTicks = 0;
//     int thresh = 150;
//     mspeed(baseSpeed, baseSpeed);
//     print("Moving forward 25 cm, target ", target);
//         Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
//     while (abs(leftTicks) < target and abs(rightTicks) < target) {
//         updateSensors();
//         Serial.printf("Dbg: L: %d, R: %d, F: %d TL: %d, TR: %d\n", distLeft, distRight, distFront, leftTicks, rightTicks);
//         // runWallPID(distLeft, distRight);
//         float isWallLeft = (distLeft < thresh);
//         float isWallRight = (distRight < thresh);
//         if (isWallLeft and isWallRight) {
//             runWallPID(distLeft, distRight);
//             Serial.println("Both walls");
//         } else if (isWallLeft) {
//             runLeftWallPID(distLeft);
//             Serial.println("Left walls");
//         } else if (isWallRight) {
//             runRightWallPID(distRight);
//             Serial.println("Right wall");
//         } else {
//             runEncoderPID();
//             Serial.println("Encoder PID");
//         }
//         delay(20);
//     }
//     mspeed(0, 0);
// }

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

    // b[0] = (dummymaze[posX][posY] & 1) == 1;
    // b[1] = (dummymaze[posX][posY] & 2) == 2;
    // b[2] = (dummymaze[posX][posY] & 4) == 4;
    // b[3] = (dummymaze[posX][posY] & 8) == 8;

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

float encoderCountToDegrees = 0.9;
void rotate(int degree) {
    int target = encoderCountToDegrees * abs(degree);
    int dir = degree > 0 ? 1 : -1;
    bool rotatingLeft = true, rotatingRight = true;
    leftTicks = rightTicks = 0;

    mspeed(dir*baseSpeed, -dir*baseSpeed);
    print("Rotating ", degree, " degrees, target ", target);
    print("Writing speeds ", dir*baseSpeed, " ", -dir*baseSpeed);
    while (rotatingLeft and rotatingRight) {
        if (abs(leftTicks) > target) {
            mspeed(0, 300);
            rotatingLeft = false;
        }
        if (abs(rightTicks) > target) {
            mspeed(300, 0);
            rotatingRight = false;
        }
        // print("Left ticks: ", leftTicks, " Right ticks: ", rightTicks);
        // delay(10);
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

    // rotate(90);
    // delay(2000);
    // rotate(-90);
    // delay(2000);
    // rotate(-180);
//   mspeed(200, 200);
    // for (int i = 0; i < 3; i++) {
    //     moveForward();  
    //     delay(3000);
    // }
    delay(1000);
    // moveDistance(25, 150);
    // Serial.printf("Reading from sensor 1 is %d\n", sensor1.read());
    // Serial.printf("Reading from sensor 2 is %d\n", sensor2.read());
    // Serial.printf("Reading from sensor 3 is %d\n", sensor3.read());
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
    delay(1000);
    moveForward();
    delay(1000);
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