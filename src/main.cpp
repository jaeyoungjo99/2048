#include <ros/ros.h>

#include "game_2048/window.hpp"

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "game_2048");
    ros::NodeHandle nh;

    // Initialize GLUT
    glutInit(&argc, argv);

    Window window_;

    window_.MainLoop();
    
    return 0;
}