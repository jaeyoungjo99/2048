#include <ros/ros.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "game_2048/config.hpp"
#include "game_2048/game_manager.hpp"

// window 좌표계는 좌상단이 0,0
// OpenGL 좌표계는 좌하단이 0,0


// Variables
Direction input_direction_ = Direction::REQUESTED;
GameManager game_manager_;


// Utils
// Function to render text on the screen
void RenderText(float x, float y, const std::string& text, float r, float g, float b) {
    glColor3f(r, g, b); // Set the text color
    glPushMatrix(); // Save the current matrix
    glTranslatef(x, y, 0); // Move to the desired position

    glRasterPos2f(0, 0); // Set the raster position to the origin
    for (char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, c); // Draw each character
    }

    glPopMatrix(); // Restore the previous matrix
}

// Function to draw the game board
void DrawGameBoard() {
    // Set the background color
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f); // Light gray background
    glClear(GL_COLOR_BUFFER_BIT); // Clear the screen with the background color

    // Set the color for the grid
    glColor3f(0.8f, 0.7f, 0.6f); // Light brown color

    // Draw the board rectangles with spacing
    for (int i = 0; i < GRID_NUM; ++i) {
        for (int j = 0; j < GRID_NUM; ++j) {
            float x = j * (BLOCK_SIZE + BLOCK_SPACING) - 0.4f; // Adjust position with spacing
            float y = i * (BLOCK_SIZE + BLOCK_SPACING) - 0.8f; // Adjust position with spacing

            glBegin(GL_QUADS);
            glVertex2f(x - BLOCK_SIZE/2, y - BLOCK_SIZE/2);
            glVertex2f(x + BLOCK_SIZE/2, y - BLOCK_SIZE/2);
            glVertex2f(x + BLOCK_SIZE/2, y + BLOCK_SIZE/2);
            glVertex2f(x - BLOCK_SIZE/2, y + BLOCK_SIZE/2);
            glEnd();
        }
    }
}


// Function to draw grid number
void DrawGridNumber(int grid[GRID_NUM][GRID_NUM]) {
    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            int grid_number = grid[i][j];
            float r, g, b; // Color variables

            // Set color based on grid_number % 9
            int color_index = grid_number % 9;

            if (color_index >= 0 && color_index <= 2) { // 0, 1, 2 -> White
                r = 1.0f; g = 1.0f; b = 1.0f; // White
            } else if (color_index >= 3 && color_index <= 5) { // 3, 4, 5 -> Orange
                r = 1.0f; g = 0.5f; b = 0.0f; // Orange
            } else { // 6, 7, 8 -> Yellow
                r = 1.0f; g = 1.0f; b = 0.0f; // Yellow
            }

            if (grid_number != 0) { // Only draw non-zero numbers
                float x = j * (BLOCK_SIZE + BLOCK_SPACING) - 0.4f; // Adjust position with spacing
                float y = i * (BLOCK_SIZE + BLOCK_SPACING) - 0.8f; // Adjust position with spacing

                // Set the color for the grid
                glColor3f(r, g, b); 

                glBegin(GL_QUADS);
                glVertex2f(x - BLOCK_SIZE/2, y - BLOCK_SIZE/2);
                glVertex2f(x + BLOCK_SIZE/2, y - BLOCK_SIZE/2);
                glVertex2f(x + BLOCK_SIZE/2, y + BLOCK_SIZE/2);
                glVertex2f(x - BLOCK_SIZE/2, y + BLOCK_SIZE/2);
                glEnd();

                int grid_number_final = pow(2, grid_number);
                
                std::string number_text = std::to_string(grid_number_final); // Convert number to string
                RenderText(x, y, number_text, 0.5, 0.5, 0.5); // Draw the number with the corresponding color
            }
        }
    }
}

void DrawScore(int score) {
    // Set the color for the text
    glColor3f(0.0f, 0.0f, 0.0f); // Black color

    std::string score_text = "Score " + std::to_string(score);
    RenderText(0.0f, 0.80f, score_text, 0.0f, 0.0f, 0.0f); // Position the text at the top-left corner

}

// Function to handle key input
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if(input_direction_ != Direction::REQUESTED) {
        std::cout << "Input direction is not requested" << std::endl;
        return;
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_UP:
                // Handle up arrow key
                std::cout << "Up key pressed" << std::endl;
                input_direction_ = Direction::UP;
                break;
            case GLFW_KEY_DOWN:
                // Handle down arrow key
                std::cout << "Down key pressed" << std::endl;
                input_direction_ = Direction::DOWN;
                break;
            case GLFW_KEY_LEFT:
                // Handle left arrow key
                std::cout << "Left key pressed" << std::endl;
                input_direction_ = Direction::LEFT;
                break;
            case GLFW_KEY_RIGHT:
                // Handle right arrow key
                std::cout << "Right key pressed" << std::endl;
                input_direction_ = Direction::RIGHT;
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);

    // Initialize ROS
    ros::init(argc, argv, "game_2048");
    ros::NodeHandle nh;

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Set OpenGL version (2.1)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    // Create window
    GLFWwindow* p_window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "game_2048", nullptr, nullptr);
    if (!p_window) {
        std::cerr << "Window Create Failed" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Set OpenGL context
    glfwMakeContextCurrent(p_window);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW Initialize Failed" << std::endl;
        return -1;
    }

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);



    // -------- Game Initialize --------
    game_manager_.InitializeGame();

    // Draw the background once before the loop
    DrawGameBoard();

    int score = 0;

    // Set the key callback
    glfwSetKeyCallback(p_window, KeyCallback);

    while (ros::ok() && !glfwWindowShouldClose(p_window)) {


        glClear(GL_COLOR_BUFFER_BIT); // Clear the screen

        // -------- Game Update --------
        
        if(input_direction_ != Direction::REQUESTED) {
            game_manager_.UpdateGame(input_direction_);
            input_direction_ = Direction::REQUESTED;
        }else{
            // std::cout << "Waiting for input" << std::endl;
        }

        int grid[GRID_NUM][GRID_NUM];
        game_manager_.GetGrid(grid);
        game_manager_.GetScore(score);

        if(game_manager_.IsGameOver()) {
            std::cout << "Game Over Your Score is " << score << std::endl;
            break;
        }

        // -------- Game Draw --------
        DrawGameBoard(); // Draw the game board
        DrawGridNumber(grid); // Draw moving elements
        DrawScore(score);

        glfwSwapBuffers(p_window);
        glfwPollEvents();
    }

    glfwDestroyWindow(p_window);
    glfwTerminate();
    
    return 0;
}