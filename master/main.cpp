// #include "master.h"
// #include "slave.h"
// #include "scara.h"
// #include <iostream>
// #include <vector>

// const int numSlaves = 4;

// int main(int argc, char* argv[]) {
//     char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
//     Master ecMaster(ifaceName, 8000);

//     if (ecMaster.connected()) {
//         std::vector <Slave> ecSlaves;
        
//         for(int i = 1; i <= numSlaves; i++) {
//             ecSlaves.push_back(Slave(ecMaster, i));
//         }

//         SCARA scaraRobot(250, 280, ecSlaves);

//         scaraRobot.initSlaves();

//         std::vector<int> velocities = {20000, 25000, 300, 500000};

//         double angle = 38.2;
//         JointAngles angles = scaraRobot.calculateJointAngles(380.0, -128.0, false);
//         std::vector<int> positions = {(int)angles.j1 * 1000, (int)angles.j2 * 1000, 300 * 1000, int(angle) * 1000};


//         scaraRobot.moveToPosT(velocities, positions);
//         return EXIT_SUCCESS;
//     }
//     else {
//         return EXIT_FAILURE;
//     }
// }

// main.cpp
#include "Camera.h"

int main() {
    Camera camera("192.168.0.101", 2006);
    camera.capture();

    return 0;
}
