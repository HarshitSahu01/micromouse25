#include <queue.h>
#include <pair.h>

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

int MAZESIZE = 8;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 1; 

void moveForward() {
    // Code to move the robot forward
}

void identifyBlock() {

}

void floodfill() {
    int targetX = MAZESIZE - 1, targetY = MAZESIZE - 1;
    memset(flood, -1, MAZESIZE* MAZESIZE);
    flood[targetX][targetY] = 0;

    queue<pair<int, int>> q;
    q.push({targetX, targetY});

    while (not q.empty()) {
        int [x, y] = q.front();
        q.pop();

        if (x > 0 and flood[x-1][y] == -1 and !maze[x-1][y]&8 == 8) {
            flood[x-1][y] = flood[x][y] + 1;
            q.push({x-1, y});
        }
        if (x < MAZESIZE - 1 and flood[x+1][y] == -1 and !maze[x+1][y]&2 == 2) {
            flood[x+1][y] = flood[x][y] + 1;
            q.push({x+1, y});
        }
        if (y > 0 and flood[x][y-1] == -1 and !maze[x][y-1]&4 == 4) {
            flood[x][y-1] = flood[x][y] + 1;
            q.push({x, y-1});
        }
        if (y < MAZESIZE - 1 and flood[x][y+1] == -1 and !maze[x][y+1]&1 == 1) {
            flood[x][y+1] = flood[x][y] + 1;
            q.push({x, y+1});
        }
    }
}

int rotationDirections[4] = { 0, 90, 180, -90 }; // 0: forward, 1: right, 2: backward, 3: left
int nextBlock() {
    int target = -1;
    if (x > 0 and flood[posX-1][posY] == flood[posX][posY] - 1) {
        target = 3; 
    } else if (x < MAZESIZE - 1 and flood[posX+1][posY] == flood[posX][posY] - 1) {
        target = 1; 
    } else if (y > 0 and flood[posX][posY-1] == flood[posX][posY] - 1) {
        target = 0; 
    } else if (y < MAZESIZE - 1 and flood[posX][posY+1] == flood[posX][posY] - 1) {
        target = 2; 
    }

    return rotationDirections[(target-orientation) % 4];

}

int encoderCountToDegrees = 0.927517;
void rotate(int degree) {
    int target = encoderCountToDegrees * abs(degree);
    int dir = degree > 0 ? 1 : -1;
    bool rotatingLeft = true, rotatingRight = true;
    leftTicks = rightTicks = 0;

    mspeed(dir*baseSpeed, -dir*baseSpeed);
    while (rotatingLeft and rotatingRight) {
        if (abs(leftTicks) > target) {
            mspeed(0, 300);
            rotatingLeft = false;
        }
        if (abs(rightTicks) > target) {
            mspeed(300, 0);
            rotatingRight = false;
        }
    }

}

void setup() {}

void loop() {
    moveForward();
    int x = identifyBlock();
    while (x) {
        Serial.println("End of the maze");
    }
    floodfill();
    rotate(nextBlock());
}

void mspeed(int a, int b) {
    if (abs(a) <= 255) {
        digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
        analogWrite(PWMA, abs(a));
    }

    if (abs(b) <= 255) {
        digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
        analogWrite(PWMB, b);
    }
}