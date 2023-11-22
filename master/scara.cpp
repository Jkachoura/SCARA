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

/**
 * Acknowledges faults and enables powerstage for all slaves.
 * 
*/
void SCARA::initSlaves() {
    for (auto& ecSlave : this->ecSlaves) {
        ecSlave.acknowledge_faults();
        ecSlave.enable_powerstage();
    } 
}

/**
 * Moves a slave to a position.
 * 
 * @param ecSlave The slave to move
 * @param slaveNr The index of the slave in the ecSlaves vector
 * @param position The position to move to
 * @param velocity The velocity to move with
 * 
*/
void SCARA::moveToPos(Slave ecSlave, int slaveNr, int position, int velocity) {
    // Ensure that slaveNr is a valid index (0 to size() - 1)
    if (slaveNr >= 0 && slaveNr < this->ecSlaves.size()) {
        this->ecSlaves[slaveNr].position_task(position, velocity, true, false);
    } else {
        // Handle error: slaveNr is out of range
        std::cerr << "Error: Invalid slave number\n";
    }
}

/**
 * Moves all slaves to a position.
 * 
 * @param velocities The velocities to move with
 * @param positions The positions to move to
 * @param startThreads2First True if the ball screw nut slaves should be moved first, false if the joints should be moved first
 * 
 * @note The number of velocities and positions must match the number of slaves
 * 
*/
void SCARA::moveToPosT(const std::vector<int>& velocities, const std::vector<int>& positions) {
    // Ensure that the number of velocities and positions match the number of slaves
    if (velocities.size() != this->ecSlaves.size()) {
        //print velocities.size() and this->ecSlaves.size()
        std::cout << "velocities.size() = " << velocities.size() << std::endl;
        std::cout << "this->ecSlaves.size() = " << this->ecSlaves.size() << std::endl;
        std::cerr << "Error: Number of velocities does not match the number of slaves\n";
        return;
    }
    moveToPos(ecSlaves[2], 2, 0, velocities[2]);
    // Ball screw nut motors
    int ballScrewNut = 2;
    // Joints motors
    int joints = 2;

    std::vector<std::thread> threads(joints);
    for (int i = 0; i < joints; ++i) {
        threads[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i], i, positions[i], velocities[i]); 
    }

    for (int i = 0; i < joints; ++i) {
        threads[i].join();
    }

    std::vector<std::thread> threads2(ballScrewNut);
    for (int i = 0; i < ballScrewNut; ++i) {
        threads2[i] = std::thread(&SCARA::moveToPos, this, ecSlaves[i + 2], i + 2, positions[i + 2], velocities[i + 2]); 
    }

    for (int i = 0; i < ballScrewNut; ++i) {
        threads2[i].join();
    }

    Sleep(5000);

    moveToPos(ecSlaves[2], 2, 0, velocities[2]);
}

/**
 * Moves the robot to a set of target positions.
 * 
 * @param velocities The velocities to move with
 * 
 * @note The set of target positions is hardcoded
*/
void SCARA::demo(const std::vector<int>& velocities){
    std::vector<std::pair<double, double>> targetPositions = {
        {244.16, -187.19}, {333.86, -186.59}, {424.23, -186.09},
        {244.58, -47.11}, {334.07, -45.80}, {423.70, -45.51},
        {244.73, 92.74}, {333.82, 93.47}, {423.77, 94.23}
    };

    std::vector<std::vector<int>> endPositions;

    // Calculate and store end positions for each target position
    for (const auto& target : targetPositions) {
        JointAngles angles = calculateJointAngles(target.first, target.second, false);
        endPositions.push_back({ (int)angles.j1 * 1000, (int)angles.j2 * 1000, 275 * 1000 });
    }

    // Move to each target position
    for (const auto& endPos : endPositions) {
        moveToPosT(velocities, endPos);
    }
}