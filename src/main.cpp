#include "game_2048/window.hpp"

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);

    Window window_;
    window_.MainLoop();

    return 0;
}