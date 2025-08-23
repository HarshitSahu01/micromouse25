// #include <Wire.h>
// #include <VL53L1X.h>
#include <iostream>
#include <queue>
#include <utility>
#include <cstring>
using namespace std;

// Utility function
void print() {
//   Serial.println();  // Print a newline at the end of the line/
    cout << '\n';
}

// Variadic template function to handle multiple arguments
template<typename T, typename... Args>
void print(T first, Args... args) {
//   Serial.print(first);
//   Serial.print(" ");
cout << first << ' ';
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
// VL53L1X sensor1;
// VL53L1X sensor2;
// VL53L1X sensor3;

// void IRAM_ATTR leftEncoderISR() {
//   bool a = digitalRead(LEFT_ENC_A);
//   bool b = digitalRead(LEFT_ENC_B);
//   leftTicks += (a == b) ? 1 : -1;
// }

// void IRAM_ATTR rightEncoderISR() {
//   bool a = digitalRead(RIGHT_ENC_A);
//   bool b = digitalRead(RIGHT_ENC_B);
//   rightTicks += (a == b) ? 1 : -1;
// }

int baseSpeed = 150;

const int MAZESIZE = 8;
int maze[MAZESIZE][MAZESIZE];
// int dummymaze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 0; 

int dummymaze[MAZESIZE][MAZESIZE] = {
    0, 2, 8, 0, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 2, 10, 8, 0, 0, 0, 0,
    0, 0, 2, 8, 0, 0, 0, 0
};

// Tune this once you calibrate encoders!
static const float TICKS_PER_CM = 11.43;   // <-- measure later
static const int   MOVE_CM       = 25;
static const int   BASE_PWM      = 150;
static const int   STOP_TOL_TICKS = 5;
static const int   FRONT_STOP_MM  = 80;    // stop if front wall closer than this

// PID gains (initial guess)
static const float KP = 0.4f;
static const float KI = 0.001f;
static const float KD = 0.1f;

// Desired wall offset (half of 25 cm cell ≈ 125 mm, we keep ~120 mm to avoid scraping)
static const int DESIRED_WALL_MM = 120;
static const int VALID_MIN_MM = 50;
static const int VALID_MAX_MM = 300;

inline bool valid_mm(int d) { return d >= VALID_MIN_MM && d <= VALID_MAX_MM; }

void moveForward() {
//   long targetTicks = (long)(MOVE_CM * TICKS_PER_CM + 0.5f);
//   leftTicks = 0;
//   rightTicks = 0;

//   // PID vars
//   float errPrev = 0, errInt = 0;
//   unsigned long tPrev = micros();

//   while (true) {
//     // timing
//     unsigned long tNow = micros();
//     float dt = (tNow - tPrev) * 1e-6f;
//     if (dt < 0.005f) continue; // run ~200Hz
//     tPrev = tNow;

//     // encoder progress
//     long L = leftTicks;
//     long R = rightTicks;
//     long avg = (L + R) / 2;
//     long remain = targetTicks - avg;

//     // stop if target reached
//     if (remain <= STOP_TOL_TICKS) break;

//     // stop if front wall too close
//     int dFront = sensor2.read();
//     if (valid_mm(dFront) && dFront < FRONT_STOP_MM) {
//       break;
//     }

//     // read side walls
//     int dLeft  = sensor1.read();
//     int dRight = sensor3.read();
//     bool hasLeft  = valid_mm(dLeft);
//     bool hasRight = valid_mm(dRight);

//     // compute lateral error
//     float e = 0.0f;
//     if (hasLeft && hasRight) {
//       // center in corridor
//       e = (float)(dRight - dLeft);
//     } else if (hasLeft) {
//       e = (float)(DESIRED_WALL_MM - dLeft);
//     } else if (hasRight) {
//       e = (float)(dRight - DESIRED_WALL_MM);
//     } else {
//       // no wall → use encoder diff
//       e = (float)(R - L);
//     }

//     // PID steering
//     errInt += e * dt;
//     float dErr = (e - errPrev) / max(dt, 1e-4f);
//     errPrev = e;
//     float steer = KP * e + KI * errInt + KD * dErr;

//     // wheel commands
//     int leftCmd  = BASE_PWM + (int)steer;
//     int rightCmd = BASE_PWM - (int)steer;

//     // clamp
//     leftCmd  = constrain(leftCmd, -255, 255);
//     rightCmd = constrain(rightCmd, -255, 255);

//     mspeed(leftCmd, rightCmd);
//   }

//   // stop motors
//   mspeed(0, 0);
//   delay(20);
}

void identifyBlock() {
    uint8_t a[4], b[4];
    int thresh = 150;
    
    // a[0] = sensor2.read() < thresh;
    // a[1] = sensor3.read() < thresh;
    // a[3] = sensor1.read() < thresh;
    // a[2] = 0;

    // for (int i = 0; i < 4; i++) {
    //     b[i] = a[(i - orientation+4) % 4];
    // }

    b[0] = dummymaze[posX][posY] & 1 == 1;
    b[1] = dummymaze[posX][posY] & 2 == 2;
    b[2] = dummymaze[posX][posY] & 4 == 4;
    b[3] = dummymaze[posX][posY] & 8 == 8;
    
    int type = 8 * b[3] + 4 * b[2] + 2 * b[1] + b[0];
    maze[posX][posY] = type;
    if (posX > 0) {
        maze[posX-1][posY] |= (b[3] ? 8 : 0); // left wall
    }
    if (posX < MAZESIZE - 1) {
        maze[posX+1][posY] |= (b[1] ? 2 : 0); // right wall
    }
    if (posY > 0) {
        maze[posX][posY-1] |= (b[0] ? 4 : 0); // front wall
    }
    if (posY < MAZESIZE - 1) {
        maze[posX][posY+1] |= (b[2] ? 1 : 0); // back wall
    }
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
    // cout << target << ' ' << rotationDirections[(orientation - oldOrient) % 4] << "Meow\n"; 
    return rotationDirections[(orientation - oldOrient+4) % 4];
}

float encoderCountToDegrees = 0.9;
void rotate(int degree) {
    cout << " Rotated bot by " << degree << '\n';
    // int target = encoderCountToDegrees * abs(degree);
    // int dir = degree > 0 ? 1 : -1;
    // bool rotatingLeft = true, rotatingRight = true;
    // leftTicks = rightTicks = 0;

    // mspeed(dir*baseSpeed, -dir*baseSpeed);
    // print("Rotating ", degree, " degrees, target ", target);
    // print("Writing speeds ", dir*baseSpeed, " ", -dir*baseSpeed);
    // while (rotatingLeft and rotatingRight) {
    //     if (abs(leftTicks) > target) {
    //         mspeed(0, 300);
    //         rotatingLeft = false;
    //     }
    //     if (abs(rightTicks) > target) {
    //         mspeed(300, 0);
    //         rotatingRight = false;
    //     }
    //     print("Left ticks: ", leftTicks, " Right ticks: ", rightTicks);
    //     delay(10);
    // }
    // mspeed(0, 0);

}

void setup() {
//     Serial.begin(115200);
//     // Use custom I2C pins: SDA = 33, SCL = 32
//   Wire.begin(33, 32);

//   // --- Step 1: Reset all sensors ---
//   pinMode(XSHUT1, OUTPUT);
//   pinMode(XSHUT2, OUTPUT);
//   pinMode(XSHUT3, OUTPUT);
//   digitalWrite(XSHUT1, LOW);
//   digitalWrite(XSHUT2, LOW);
//   digitalWrite(XSHUT3, LOW);
//   delay(100);

//   // --- Step 2: Boot sensor1 only ---
//   pinMode(XSHUT1, INPUT); // release -> pulled HIGH
//   delay(10);
//   if (!sensor1.init()) {
//     Serial.println("Sensor1 init failed!");
//     while (1);
//   }
//   sensor1.setAddress(ADDR1);

//   // --- Step 3: Boot sensor2 ---
//   pinMode(XSHUT2, INPUT);
//   delay(10);
//   if (!sensor2.init()) {
//     Serial.println("Sensor2 init failed!");
//     while (1);
//   }
//   sensor2.setAddress(ADDR2);

//   // --- Step 4: Boot sensor3 ---
//   pinMode(XSHUT3, INPUT);
//   delay(10);
//   if (!sensor3.init()) {
//     Serial.println("Sensor3 init failed!");
//     while (1);
//   }
//   sensor3.setAddress(ADDR3);

//   // --- Step 5: Configure all sensors ---
//   sensor1.setDistanceMode(VL53L1X::Medium);
//   sensor1.setMeasurementTimingBudget(33000); // 33 ms
//   sensor1.startContinuous(33);

//   sensor2.setDistanceMode(VL53L1X::Medium);
//   sensor2.setMeasurementTimingBudget(33000);
//   sensor2.startContinuous(33);

//   sensor3.setDistanceMode(VL53L1X::Medium);
//   sensor3.setMeasurementTimingBudget(33000);
//   sensor3.startContinuous(33);

//   Serial.println("All sensors initialized.");
//     // Motor setup
//   pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
//   pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
//   pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

//   // Encoder setup
//   pinMode(LEFT_ENC_A, INPUT_PULLUP);
//   pinMode(LEFT_ENC_B, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_A, INPUT_PULLUP);
//   pinMode(RIGHT_ENC_B, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
//   Serial.println("Motor Encoders initialised");
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
    // delay(1000);
}

void loop() {
    cout << "\nCurrently at " << posX << ' ' << posY << ' ' << orientation << '\n';
    identifyBlock();
    floodfill();

    // for (int i = 0; i < MAZESIZE; i++) {
    //     for (int j = 0; j < MAZESIZE; j++) {
    //         cout << flood[i][j] << ' ';
    //     }
    //     cout << '\n';
    // }

    int degrees = nextBlock();
    while (degrees == -1) {
        cout << "End of maze";
        while (1) {}
        // Serial.println("End of the maze");
        // delay(1000);
    }
    rotate(degrees);
    moveForward();
}

int main() {
    // while(1) {loop();}
    // nextBlock();
    //     floodfill();
    // cout << "HEre\n";

    // for (int i = 0; i < MAZESIZE; i++) {
    //     for (int j = 0; j < MAZESIZE; j++) {
    //         cout << flood[i][j] << ' ';
    //     }
    //     cout << '\n';
    // }
    for (int i = 0; i <100; i++) {
        loop();
    }

}

void mspeed(int a, int b) {
    // if (abs(a) <= 255) {
    //     digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
    //     analogWrite(PWMA, abs(a));
    // }

    // if (abs(b) <= 255) {
    //     digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
    //     analogWrite(PWMB, abs(b));
    // }
}