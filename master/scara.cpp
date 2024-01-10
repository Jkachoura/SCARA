// scara.cpp
#include "scara.h"

const double PI = 3.14159265358979323846;

/**
 * Constructor for SCARA.
 * 
 * @param length_a1 The length of the first arm
 * @param length_a2 The length of the second arm
 * @param ecSlavesVec A vector containing all the slaves
 * @param airPressureSlave The index of the slave that controls the air pressure
 * 
*/
SCARA::SCARA(double length_a1, double length_a2, std::vector<Slave>& ecSlavesVec, int airPressureSlave) : a1(length_a1), a2(length_a2), ecSlaves(ecSlavesVec), apSlave(airPressureSlave) {
    initSlaves();

    // Set the bitmask for the air pressure slave
    uint32_t bitmask = 196608;
    ecSlaves[airPressureSlave - 1].write_sdo(0x60FE, 0x02, &bitmask, sizeof(bitmask));     
}

/**
 * Destructor for SCARA.
*/
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
JointAngles SCARA::calculateJointAngles(double x, double y, double angle, bool elbowLeft) {
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

    result.gripper_angle = result.j1 + result.j2 - angle;
    //print gripper angle and j1 and j2
    std::cout << "gripper angle: " << result.gripper_angle << std::endl;
    std::cout << "j1: " << result.j1 << std::endl;
    std::cout << "j2: " << result.j2 << std::endl;

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
    if (slaveNr >= 1 && slaveNr <= this->ecSlaves.size()) {
        this->ecSlaves[slaveNr - 1].position_task(position, velocity, true, false);
    } else {
        // Handle error: slaveNr is out of range
        std::cerr << "Error: Invalid slave number\n";
    }
}

/**
 * Moves Joint 1 and Joint 2 to a set of target positions.
 * 
 * @param j1 The target position for Joint 1
 * @param j2 The target position for Joint 2
 * @param velocityj1 The velocity to move Joint 1 with
 * @param velocityj2 The velocity to move Joint 2 with
 * 
*/
void SCARA::moveJ1J2(int j1, int j2, int velocityj1, int velocityj2) {
    std::vector<std::thread> threads(2);

    // Move slave for joint 1 to position j1
    threads[0] = std::thread(&SCARA::moveToPos, this, ecSlaves[0], 1, j1, velocityj1);

    // Move slave for joint 2 to position j2
    threads[1] = std::thread(&SCARA::moveToPos, this, ecSlaves[1], 2, j2, velocityj2);

    // Join both threads to wait for them to finish
    for (int i = 0; i < 2; ++i) {  // Use the size of the threads vector
        threads[i].join();
    }
}

/**
 * Moves Joint 3 and Joint 4 to a set of target positions.
 * 
 * @param j3 The target position for Joint 3
 * @param j4 The target position for Joint 4
 * @param velocityj3 The velocity to move Joint 3 with
 * @param velocityj4 The velocity to move Joint 4 with
 * 
*/
void SCARA::moveJ3J4(int j3, int j4, int velocityj3, int velocityj4) {
    std::vector<std::thread> threads(2);

    // Move slave for joint 3 to position j3
    threads[0] = std::thread(&SCARA::moveToPos, this, ecSlaves[2], 3, j3, velocityj3);

    // Move slave for joint 4 to position j4
    threads[1] = std::thread(&SCARA::moveToPos, this, ecSlaves[3], 4, j4, velocityj4);

    // Join both threads to wait for them to finish
    for (int i = 0; i < 2; ++i) {  // Use the size of the threads vector
        threads[i].join();
    }
}

/**
 * Moves the robot to the home position.
 * 
 * @note The home position is defined as Joint 1 being at -90 degrees and Joint 2 being at 0 degrees.
 * this is so the camera can see the batteries/objects.
 * 
*/
void SCARA::moveTo0() {
    moveJ3J4(0, 0, j3speed, j4speed); // Spindel-axis and rotation-axis to 0 degrees so when moving nothing hits anything
    moveJ1J2(-90000, 0, j1speed, j2speed); // -90 degrees for joint 1, 0 degrees for joint 2
    
}

/**
 * Picks up an object.
 * 
 * @param x The x coordinate of the object
 * @param y The y coordinate of the object
 * @param angle The angle of the object
 * @param elbowLeft True if the elbow is on the left side of the robot, false if it is on the right
 * 
*/
void SCARA::pickUp(double x, double y, double angle, bool elbowLeft) {
    JointAngles angles = calculateJointAngles(x, y, angle, elbowLeft);

    int j1pos = (int)angles.j1 * 1000; 
    int j2pos = (int)angles.j2 * 1000;
    int anglepos = (int)angles.gripper_angle * 1000;

    // Calculate the offset for the pick up position based on the x coordinate
    // In the searchfield the depth decreases by 4mm for every 340mm
    double offset= (4.0/(340.0)) * (490 - x);
    double picklt = 293 + offset;

    // Because of the spindle-axis the spindle changes 16mm for every 360 degrees rotation
    double off = abs(angle) * (16.0/360.0);

    // If the angle is positive, the spindle-axis moves down, if it is negative, the spindle-axis moves up
    if(angle > 0) {
        picklt = picklt - off; 
    } else {
        picklt = picklt + off;
    }

    int apickupl = (int)picklt * 1000;

    // Move to the pick up position
    moveJ1J2(j1pos, j2pos, j1speed, j2speed);

    // Move the spindle-axis to the pick up position
    moveJ3J4(apickupl, anglepos,  j3speed, j4speed);
    
    airPressureOn();

    Sleep(500);

    // Wait for the vacuum to turn on
    // This is done by checking the vacuum value every 100ms for 3 seconds
    // If there is no vacuum after 3 seconds, the robot will continue
    auto startTime = std::chrono::high_resolution_clock::now();
    while (!getVacuum()) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

        std::cout << "Waiting for vacuum" << std::endl;

        // Break out of the loop if more than 3 seconds have passed
        if (elapsedTime > 1.5) {
            std::cout << "Timeout: Unable to detect vacuum within 3 seconds." << std::endl;
            break;
        }
        // Sleep for a short duration to avoid high CPU usage in the loop
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Move the spindle-axis back up
    moveJ3J4(0, anglepos, j3speed, j4speed);
}

/**
 * Drops an object.
 * 
 * @param elbowLeft True if the elbow is on the left side of the robot, false if it is on the right
 * 
 * @note the drop position is defined at x = 248.7, y = -381.9, angle = 70.0
 */
void SCARA::drop(bool elbowLeft){
    JointAngles dropangles = calculateJointAngles(248.7, -381.9, -70.0, elbowLeft);

    int j1droppos = (int)dropangles.j1 * 1000;
    int j2droppos = (int)dropangles.j2 * 1000;
    int droppangle = (int)dropangles.gripper_angle * 1000;

    moveJ1J2(j1droppos, j2droppos, j1speed, j2speed);

    moveJ3J4(dropl, droppangle, j3speed, j4speed);

    airPressureOff();

    while(getVacuum()){
        std::cout << "Waiting for vacuum to turn off" << std::endl;
    }

    moveJ3J4(0, droppangle, j3speed, j4speed);
}
/**
 * Turns on the air pressure.
 * 
*/
void SCARA::airPressureOn(){
    uint32_t airpressureon = 65536;
    ecSlaves[this->apSlave - 1].write_sdo(0x60FE, 0x01, &airpressureon, sizeof(airpressureon));
}

/**
 * Turns off the air pressure.
 * 
*/
void SCARA::airPressureOff(){
    uint32_t airpressureoff = 0;
    ecSlaves[this->apSlave - 1].write_sdo(0x60FE, 0x01, &airpressureoff, sizeof(airpressureoff));
}

/**
 * Checks if the vacuum is created or not.
 * 
 * @return True if the vacuum is created, false if it is off
 * 
*/
bool SCARA::getVacuum(){
    uint32_t vacuum;
    int vacuumsize = sizeof(vacuum);
    ecSlaves[this->apSlave - 1].read_sdo(0x213D, 0x01, &vacuum, &vacuumsize);

    // This number is based on the binary value of the I/O pins on the slave
    if(vacuum == 12603140){
        return false;
    }
    else if(vacuum == 12611332){
        return true;
    }
    else{
        std::cout << "Error: Invalid vacuum value" << std::endl;
        return false;
    }

}