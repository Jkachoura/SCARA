// scara.cpp
#include "scara.h"

const double PI = 3.14159265358979323846;

SCARA::SCARA(double length_a1, double length_a2) : a1(length_a1), a2(length_a2) {}

SCARA::~SCARA() {}

JointAngles SCARA::calculateJointAngles(double x, double y, bool elbowLeft) {
    double c2 = (pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);
    double j2_neg = atan2(-sqrt(1 - pow(c2, 2)), c2);
    double j2_pos = atan2(sqrt(1 - pow(c2, 2)), c2);
    double j2 = elbowLeft ? j2_neg : j2_pos;
    double atan1 = atan2(y, x);
    double atan2_1 = atan2(a2 * sin(j2), a1 + a2 * c2);
    double j1 = atan1 - atan2_1;

    JointAngles result;
    result.j1 = j1 * 180.0 / PI; // Convert J1 to degrees
    result.j2 = j2 * 180.0 / PI; // Convert J2 to degrees

    return result;
}
