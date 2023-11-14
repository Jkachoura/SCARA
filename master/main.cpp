// main.cpp
#include "scara.h"
#include <iostream>

int main() {
    // Create an instance of the SCARA class with specific arm lengths
    SCARA scaraRobot(250, 280);

    // Use the SCARA class to calculate joint angles
    JointAngles angles = scaraRobot.calculateJointAngles(238, 380, false);

    // Output the results
    std::cout << "Joint 1 Angle: " << angles.j1 << " degrees\n";
    std::cout << "Joint 2 Angle: " << angles.j2 << " degrees\n";

    return 0;
}
