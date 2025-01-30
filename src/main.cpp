#include <ros/ros.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "game_2048/config.hpp"

// window 좌표계는 좌상단이 0,0
// OpenGL 좌표계는 좌하단이 0,0

// Function to draw the game board
void DrawGameBoard() {
    // Set the background color
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f); // Light gray background
    glClear(GL_COLOR_BUFFER_BIT); // Clear the screen with the background color

    // Set the color for the grid
    glColor3f(0.8f, 0.7f, 0.6f); // Light brown color

    // Draw the board rectangles with spacing
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float x = j * 0.25f - 0.4f; // Adjust position with spacing
            float y = i * 0.25f - 0.8f; // Adjust position with spacing

            glBegin(GL_QUADS);
            glVertex2f(x, y);
            glVertex2f(x + 0.2f, y);
            glVertex2f(x + 0.2f, y + 0.2f);
            glVertex2f(x, y + 0.2f);
            glEnd();
        }
    }
}

// Function to draw moving elements
void DrawMovingElements() {
    // Example: Draw a moving square
    glColor3f(1.0f, 0.0f, 0.0f); // Red color
    float moving_x = 0.0f; // Example position
    float moving_y = 0.0f; // Example position

    glBegin(GL_QUADS);
    glVertex2f(moving_x - 0.05f, moving_y - 0.05f);
    glVertex2f(moving_x + 0.05f, moving_y - 0.05f);
    glVertex2f(moving_x + 0.05f, moving_y + 0.05f);
    glVertex2f(moving_x - 0.05f, moving_y + 0.05f);
    glEnd();
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

    std::cout << "2048 Game Start" << std::endl;

    // Draw the background once before the loop
    DrawGameBoard();

    while (ros::ok() && !glfwWindowShouldClose(p_window)) {
        glClear(GL_COLOR_BUFFER_BIT); // Clear the screen

        // Draw the fixed background
        DrawGameBoard(); // Draw the game board

        // Draw moving elements on top of the background
        DrawMovingElements(); // Draw moving elements

        glfwSwapBuffers(p_window);
        glfwPollEvents();
    }

    glfwDestroyWindow(p_window);
    glfwTerminate();
    
    return 0;
}