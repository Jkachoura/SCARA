// scara.h
#ifndef SCARA_H
#define SCARA_H

#include <cmath>
#include <vector>
#include "slave.h"
#include "master.h"
#include <thread>
#include <chrono>

struct JointAngles {
    double j1;
    double j2;
    double gripper_angle;
};

class SCARA {
    private:
        double a1;  // Length of the first arm
        double a2;  // Length of the second arm
        std::vector<Slave>& ecSlaves;
        int apSlave;
        void moveToPos(Slave ecSlave, int slaveNr, int position, int velocity);
        void initSlaves();
        
    public:
        SCARA(double length_a1, double length_a2, std::vector<Slave>& ecSlavesVec, int airPressureSlave);
        ~SCARA();
        JointAngles calculateJointAngles(double x, double y, double angle, bool elbowLeft);
        void pickUp(double x, double y, double angle, bool elbowLeft);
        void airPressureOn();
        void airPressureOff();
        bool getVacuum();
        void moveJ1J2(int j1, int j2, int velocityj1, int velocityj2);
        void moveJ3J4(int j3, int j4, int velocityj3, int velocityj4);
        void moveTo0();
        void drop(bool elbowLeft);

    typedef enum {
        dropl = 170 * 1000, // Length of the spindel-axis to drop battery 168 mm * 1000
        j1speed = 200 * 1000, // Speed of joint 1 25 degrees per second * 1000
        j2speed = 250 * 1000, // Speed of joint 2 50 degrees per second * 1000
        j3speed = 400, // Speed of joint 3 0.1 meters per second * 1000
        j4speed = 1440 * 1000, // Speed of joint 4 36 degrees per second * 1000
        // Everything is * 1000 becasue of the fieldbus settings in the slaves
    }macros_t;
};

#endif // SCARA_H