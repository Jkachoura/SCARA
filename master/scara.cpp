// scara.cpp
#include "scara.h"

const double PI = 3.14159265358979323846;

SCARA::SCARA(double length_a1, double length_a2, std::vector<Slave>& ecSlavesVec) : a1(length_a1), a2(length_a2), ecSlaves(ecSlavesVec) {}

SCARA::~SCARA() {}

/**
 * Calculates the joint angles for the SCARA robot.
 * 
 * @param x The x coordinate of the end effector
 * @param y The y coordinate of the end effector
 * @param elbowLeft True if the elbow is on the left side of the robot, false if it is on the right
 * 
 * @return The joint angles for the SCARA robot
 * 
 */
JointAngles SCARA::calculateJointAngles(double x, double y, bool elbowLeft) {
    // Calculate c2
    double c2 = (pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);

    // Calculate J2
    double j2_neg = atan2(-sqrt(1 - pow(c2, 2)), c2);
    double j2_pos = atan2(sqrt(1 - pow(c2, 2)), c2);

    // Pick the correct J2 based on elbowLeft
    double j2 = elbowLeft ? j2_neg : j2_pos;

    // Calculate J1
    double atan1 = atan2(y, x);
    double atan2_1 = atan2(a2 * sin(j2), a1 + a2 * c2);
    double j1 = atan1 - atan2_1;

    JointAngles result;
    result.j1 = j1 * 180.0 / PI; // Convert J1 to degrees
    result.j2 = j2 * 180.0 / PI; // Convert J2 to degrees

    return result;
}

void SCARA::initSlaves() {
    for (auto& ecSlave : this->ecSlaves) {
        ecSlave.acknowledge_faults();
        ecSlave.enable_powerstage();
    } 
}

void SCARA::moveToPos(Slave ecSlave, int slaveNr, int position, int velocity) {
    // Ensure that slaveNr is a valid index (0 to size() - 1)
    if (slaveNr >= 0 && slaveNr < this->ecSlaves.size()) {
        this->ecSlaves[slaveNr].position_task(position, velocity, true, false);
    } else {
        // Handle error: slaveNr is out of range
        std::cerr << "Error: Invalid slave number\n";
    }
}

void SCARA::moveToPosT(const std::vector<int>& velocities, const std::vector<int>& positions, bool startThreads2First) {
    if (velocities.size() != this->ecSlaves.size()) {
        // Handle error: The number of velocities should match the number of slaves
        std::cerr << "Error: Number of velocities does not match the number of slaves\n";
        return;
    }

    int ballScrewNut = 1;

    // Choose the order of thread start based on the boolean parameter
    if (startThreads2First) {
        // Create and start threads2 using vector instead of an array
        std::vector<std::thread> threads2(ballScrewNut);
        for (int i = 0; i < ballScrewNut; ++i) {
            threads2[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i + 2], i + 2, positions[i + 2], velocities[i + 2]); // Adjust velocity as needed
        }

        // Wait for all threads2 to finish
        for (int i = 0; i < ballScrewNut; ++i) {
            threads2[i].join();
        }

        // Create and start threads using vector instead of an array
        int joints = 2;
        std::vector<std::thread> threads(joints);
        for (int i = 0; i < joints; ++i) {
            threads[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i], i, positions[i], velocities[i]); // Adjust velocity as needed
        }

        // Wait for all threads to finish
        for (int i = 0; i < joints; ++i) {
            threads[i].join();
        }
    } else {
        // Create and start threads using vector instead of an array
        int joints = 2;
        std::vector<std::thread> threads(joints);
        for (int i = 0; i < joints; ++i) {
            threads[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i], i, positions[i], velocities[i]); // Adjust velocity as needed
        }

        // Wait for all threads to finish
        for (int i = 0; i < joints; ++i) {
            threads[i].join();
        }

        // Create and start threads2 using vector instead of an array
        std::vector<std::thread> threads2(ballScrewNut);
        for (int i = 0; i < ballScrewNut; ++i) {
            threads2[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i + 2], i + 2, positions[i + 2], velocities[i + 2]); // Adjust velocity as needed
        }

        // Wait for all threads2 to finish
        for (int i = 0; i < ballScrewNut; ++i) {
            threads2[i].join();
        }
    }
}

