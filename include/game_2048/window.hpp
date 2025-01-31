#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "game_2048/config.hpp"
#include "game_2048/game_manager.hpp"


class Window {
public:
    void MainLoop();
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

private:
    void DrawGameBoard();
    void DrawGridCell(int grid[GRID_NUM][GRID_NUM]);
    void DrawScore(int score);
    void DrawGameOver();

private:
    void RenderText(float x, float y, const std::string& text, 
        float r, float g, float b, int font = 0);

private:
    // Variables
    Direction input_direction_ = Direction::READY;
    bool press_enter_ = false;
    GameManager game_manager_;

    int grid_prior_[GRID_NUM][GRID_NUM];

};
