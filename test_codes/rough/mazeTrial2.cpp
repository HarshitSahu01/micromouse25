
#include <queue>
#include <utility>
#include <iostream>
#include <cstring>
#include <time.h>
using namespace std;

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


int baseSpeed = 150;

const int MAZESIZE = 5;
int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2; 

// int dummymaze[MAZESIZE][MAZESIZE] = {
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0
// };


void showMaze() {
    for (int i = 0; i < MAZESIZE; i++) {
        for (int j = 0; j < MAZESIZE; j++) {
            cout << maze[i][j] << ' ';
        }
        cout << '\n';
    }
}

void showFlood() {
    for (int i = 0; i < MAZESIZE; i++) {
        for (int j = 0; j < MAZESIZE; j++) {
            cout << flood[i][j] << ' ';
        }
        cout << '\n';
    }
}

int dummymaze[MAZESIZE][MAZESIZE] = {
    11, 13, 3, 9, 3,
    8, 1, 0, 4, 6,
    10, 10, 8, 5, 3,
    10, 8, 0, 3, 10,
    14, 12, 6, 14, 14
};

void moveForward() {
    // moveDistance(25, 150);
    print("Moved 25 cms");
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

    b[0] = (dummymaze[posX][posY] & 1) == 1;
    b[1] = (dummymaze[posX][posY] & 2) == 2;
    b[2] = (dummymaze[posX][posY] & 4) == 4;
    b[3] = (dummymaze[posX][posY] & 8) == 8;

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
    printf("(%d, %d) is of type %d\n", posX, posY, maze[posX][posY]);
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
    print("Rotating ", degree, " degrees, target ");
    // print("Writing speeds ", dir*baseSpeed, " ", -dir*baseSpeed);
}


void loop() {
    printf("Bot at (%d, %d), orient: %d \n", posX, posY, orientation);
    identifyBlock();
    floodfill();
    printf("\n");
    showMaze();
    printf("\n");
    showFlood();
    int degrees = nextBlock();
    while (degrees == -1) {
        printf("End of the maze\n");
        while (1) {}
    }
    rotate(degrees);
    moveForward();
}

int main() {
    for (int i = 0; i < 100; i++) {
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