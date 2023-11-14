// scara.h
#ifndef SCARA_H
#define SCARA_H

#include <cmath>

struct JointAngles {
    double j1;
    double j2;
};

class SCARA {
private:
    double a1;  // Length of the first arm
    double a2;  // Length of the second arm

public:
    SCARA(double length_a1, double length_a2);
    ~SCARA();
    JointAngles calculateJointAngles(double x, double y, bool elbowLeft);
};

#endif // SCARA_H
