// scara.h
#ifndef SCARA_H
#define SCARA_H

#include <cmath>
#include <vector>
#include "slave.h"
#include "master.h"
#include <thread>

struct JointAngles {
    double j1;
    double j2;
};

class SCARA {
    private:
        double a1;  // Length of the first arm
        double a2;  // Length of the second arm
        std::vector<Slave>& ecSlaves;

        

    public:
        SCARA(double length_a1, double length_a2, std::vector<Slave>& ecSlavesVec);
        ~SCARA();
        JointAngles calculateJointAngles(double x, double y, bool elbowLeft);
        void initSlaves();
        void moveToPos(Slave ecSlave, int slaveNr, int position, int velocity);
        void moveToPosT(const std::vector<int>& velocities, const std::vector<int>& positions, bool startThreads2First);
};

#endif // SCARA_H