#include "master.h"
#include "slave.h"
#include "scara.h"
#include <iostream>
#include <vector>

const int numSlaves = 3;

int main(int argc, char* argv[]) {
    char ifaceName[] = "\\Device\\NPF_{DEA85026-34BA-4C8B-9840-A3CE7793A348}";
    Master ecMaster(ifaceName, 8000);

    if (ecMaster.connected()) {
        std::vector <Slave> ecSlaves;
        
        for(int i = 1; i <= numSlaves; i++) {
            ecSlaves.push_back(Slave(ecMaster, i));
        }

        SCARA scaraRobot(250, 280, ecSlaves);

        scaraRobot.initSlaves();

        // Specify individual velocities for each drive
        std::vector<int> velocities = {40000, 60000, 400}; // Adjust as needed
        std::vector<int> StartPositions = {0, 0, 0};
        
        // Start threads for moveToStart with individual velocities
        scaraRobot.moveToPosT(velocities, StartPositions, true);
        
        JointAngles angles = scaraRobot.calculateJointAngles(423.70, -45.51, false);
        
        std::vector<int> endPositions = {(int)angles.j1 * 1000, (int)angles.j2 * 1000, 275 * 1000};
        Sleep(3000);
   
        scaraRobot.moveToPosT(velocities, endPositions, false);

        Sleep(1000);

        // Start threads for moveToStart with individual velocities
        scaraRobot.moveToPosT(velocities, StartPositions, true);

        Sleep(1000);

        return EXIT_SUCCESS;
    }
    else {
        return EXIT_FAILURE;
    }
}

