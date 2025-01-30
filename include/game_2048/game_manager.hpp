#ifndef GAME_2048_GAME_MANAGER_HPP
#define GAME_2048_GAME_MANAGER_HPP

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <chrono>
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <cmath>
#include "game_2048/config.hpp"

// Type definition
typedef enum {
    READY = 0,
    REQUESTED,
    UP,
    DOWN,
    LEFT,
    RIGHT
} Direction;

class GameManager {
public:
    void InitializeGame();
    void UpdateGame(Direction direction);

    void RandomGenerate();

    bool IsGameOver();


    void MoveUp();
    void MoveDown();
    void MoveLeft();
    void MoveRight();

    
    void GetGrid(int grid[GRID_NUM][GRID_NUM]);
    void GetScore(int& score);

private:
    int grid_[GRID_NUM][GRID_NUM];
};

#endif // GAME_2048_GAME_MANAGER_HPP