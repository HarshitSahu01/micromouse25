#include <iostream>
#include <queue>

#define MAZE_SIZE 8

// Maze initialization
// Top, right, bottom, left -> abcd (binary) -> block type
// 0 empty, 15 all side walls. 
int maze[MAZE_SIZE][MAZE_SIZE] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

int flood[MAZE_SIZE][MAZE_SIZE];

// In BFS
void floodFill(int x, int y, int fillValue) {
    
}

void printFlood() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            if (flood[i][j] == 0) {
                std::cout << ". ";
            } else {
                std::cout << flood[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }
}

int main() {
    // Initialize flood array
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            flood[i][j] = 0;
        }
    }

    // Start flood fill from the starting point (0, 0)
    floodFill(0, 0, 1);

    // Print the flood filled maze
    printFlood();

    return 0;
}



