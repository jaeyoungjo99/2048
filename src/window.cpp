#include "game_2048/window.hpp"

void Window::MainLoop() {

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
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
        return;
    }

    // Set OpenGL context
    glfwMakeContextCurrent(p_window);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW Initialize Failed" << std::endl;
        return;
    }

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Draw the background once before the loop
    DrawGameBoard();

    int score = 0;

    // Set the user pointer to the current instance
    glfwSetWindowUserPointer(p_window, this); // Ensure this is set before setting the key callback


    // Set the key callback
    glfwSetKeyCallback(p_window, KeyCallback);
    
    game_manager_.InitializeGame();
    input_direction_ = Direction::REQUESTED;

    while (!glfwWindowShouldClose(p_window)) {

        if(game_manager_.IsGameOver()) {
            if(press_enter_ == false) {
                // std::cout << "Waiting for enter key" << std::endl;
            }
            else{
                press_enter_ = false;
                game_manager_.InitializeGame();
                input_direction_ = Direction::REQUESTED;
            }
        }
        else{

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
                DrawGameOver();
                input_direction_ = Direction::READY;
            }

            // -------- Game Draw --------

            if(game_manager_.IsGameOver() == false) {
                glClear(GL_COLOR_BUFFER_BIT); // Clear the screen
                DrawGameBoard(); // Draw the game board
                DrawGridCell(grid); // Draw moving elements
                DrawScore(score);
            }
        }

        glfwSwapBuffers(p_window);
        glfwPollEvents();
    }

    glfwDestroyWindow(p_window);
    glfwTerminate();
    
    return;
}

void Window::DrawGameBoard() {
    // Set the background color
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f); // Light gray background
    glClear(GL_COLOR_BUFFER_BIT); // Clear the screen with the background color

    // Draw the board

    float board_length = BLOCK_SIZE * GRID_NUM + BLOCK_SPACING * (GRID_NUM + 1);

    glColor3f(0.4f, 0.25f, 0.13f); // Dark brown color
    glBegin(GL_QUADS);
    glVertex2f(BOARD_CENTER_X - board_length / 2, BOARD_CENTER_Y - board_length / 2);
    glVertex2f(BOARD_CENTER_X + board_length / 2, BOARD_CENTER_Y - board_length / 2);
    glVertex2f(BOARD_CENTER_X + board_length / 2, BOARD_CENTER_Y + board_length / 2);
    glVertex2f(BOARD_CENTER_X - board_length / 2, BOARD_CENTER_Y + board_length / 2);
    glEnd();

    // Set the color for the grid
    glColor3f(0.8f, 0.7f, 0.6f); // Light brown color

    // Draw the board rectangles with spacing
    for (int i = 0; i < GRID_NUM; ++i) {
        for (int j = 0; j < GRID_NUM; ++j) {

            float x = (j-GRID_NUM/2 +0.5) * (BLOCK_SIZE + BLOCK_SPACING) + BOARD_CENTER_X;
            float y = (i-GRID_NUM/2 +0.5) * (BLOCK_SIZE + BLOCK_SPACING) + BOARD_CENTER_Y;

            glBegin(GL_QUADS);
            glVertex2f(x - BLOCK_SIZE/2, y - BLOCK_SIZE/2);
            glVertex2f(x + BLOCK_SIZE/2, y - BLOCK_SIZE/2);
            glVertex2f(x + BLOCK_SIZE/2, y + BLOCK_SIZE/2);
            glVertex2f(x - BLOCK_SIZE/2, y + BLOCK_SIZE/2);
            glEnd();
        }
    }
}

void Window::DrawGridCell(int grid[GRID_NUM][GRID_NUM]) {
    for(int i = 0; i < GRID_NUM; i++) {
        for(int j = 0; j < GRID_NUM; j++) {
            int grid_number = grid[i][j];
            float r, g, b; // Color variables
            float text_r, text_g, text_b;

            if (grid_number != 0) { // Only draw non-zero numbers

                // Set color based on grid_number % 9
                int color_index = grid_number % 9;

                if (color_index >= 0 && color_index <= 2) { // 0, 1, 2 -> White
                    r = 1.0f; g = 1.0f; b = 1.0f; // White
                    text_r = 0.0f; text_g = 0.0f; text_b = 0.0f; // Black

                    r -= 0.05* color_index;
                    g -= 0.05* color_index;
                    b -= 0.05* color_index;
                } else if (color_index >= 3 && color_index <= 5) { // 3, 4, 5 -> Orange
                    r = 1.0f; g = 0.5f; b = 0.0f; // Orange
                    text_r = 0.9f; text_g = 0.9f; text_b = 0.9f; // White

                    r -= 0.05* color_index;
                    g -= 0.05* color_index;

                } else { // 6, 7, 8 -> Yellow
                    r = 1.0f; g = 1.0f; b = 0.0f; // Yellow
                    text_r = 0.9f; text_g = 0.9f; text_b = 0.9f; // White

                    r -= 0.05* color_index;
                    g -= 0.05* color_index;
                }

                float x = (j-GRID_NUM/2 +0.5) * (BLOCK_SIZE + BLOCK_SPACING) + BOARD_CENTER_X;
                float y = (i-GRID_NUM/2 +0.5) * (BLOCK_SIZE + BLOCK_SPACING) + BOARD_CENTER_Y;

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
                RenderText(x, y, number_text, text_r, text_g, text_b); // Draw the number with the corresponding color
            }
        }
    }
}

void Window::DrawScore(int score) {
    // Set the color for the text
    glColor3f(0.0f, 0.0f, 0.0f); // Black color

    std::string score_text = "Score " + std::to_string(score);
    RenderText(0.0f, 0.80f, score_text, 0.0f, 0.0f, 0.0f); // Position the text at the top-left corner

}

void Window::DrawGameOver() {
    glColor3f(0.0f, 0.0f, 0.0f); // Black color

    std::string game_over_text = "Game Over. Press Enter to Restart.";
    RenderText(0.0f, 0.90f, game_over_text, 0.0f, 0.0f, 0.0f); // Position the text at the top-left corner
}



void Window::RenderText(float x, float y, const std::string& text, 
        float r, float g, float b, int font) {
    glColor3f(r, g, b); // Set the text color
    glPushMatrix(); // Save the current matrix

    float average_char_width = 0;
    float text_width = 0;
    float text_height = 0;

    if(font == 0) {
        average_char_width = 15; // Average width of a character (adjust as needed)
        text_height = 20;
    }
    else{
        average_char_width = 20; // Average width of a character (adjust as needed)
        text_height = 30;
    }
    text_width = text.length() * average_char_width; // Total width based on character count

    float gl_text_width = text_width / WINDOW_WIDTH;
    float gl_text_height = text_height / WINDOW_HEIGHT;

    // Move to the desired position, adjusting for the text width
    glTranslatef(x - gl_text_width / 2, y - gl_text_height / 2, 0); // Center the text horizontally

    glRasterPos2f(0, 0); // Set the raster position to the origin
    for (char c : text) {
        if(font == 0) {
            glutBitmapCharacter(GAME_FONT_SMALL, c); // Draw each character
        }
        else if(font == 1) {
            glutBitmapCharacter(GAME_FONT_LARGE, c); // Draw each character
        }
    }

    glPopMatrix(); // Restore the previous matrix
}

void Window::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    Window* instance = static_cast<Window*>(glfwGetWindowUserPointer(window));
    if (!instance) {
        std::cerr << "Instance is null!" << std::endl; // 디버깅 로그
        return;
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_UP:
                std::cout << "Up key pressed" << std::endl; // 디버깅 로그
                instance->input_direction_ = Direction::UP;
                break;
            case GLFW_KEY_DOWN:
                instance->input_direction_ = Direction::DOWN;
                break;
            case GLFW_KEY_LEFT:
                instance->input_direction_ = Direction::LEFT;
                break;
            case GLFW_KEY_RIGHT:
                instance->input_direction_ = Direction::RIGHT;
                break;
            case GLFW_KEY_ENTER:
                instance->press_enter_ = true;
                break;
            default:
                break;
        }
    }
}